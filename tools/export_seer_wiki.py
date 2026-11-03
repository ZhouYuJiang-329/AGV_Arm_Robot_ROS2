#!/usr/bin/env python3
"""Export readable Feishu wiki pages using a separately logged-in Firefox."""

import argparse
import hashlib
import json
import re
import shutil
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from urllib.parse import urljoin, urlsplit


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URL = 'https://seer-group.feishu.cn/wiki/ZQ8gwqnvKiNklUkiqtCci7gonLd'

# Only read rendered content and links. Never request cookies or private APIs.
SNAPSHOT = r"""
const selectors = ['.docx-page-block', '.doc-content', '.wiki-doc-content',
                   '[data-testid="doc-content"]', '[role="main"]', 'main'];
const candidates = selectors.flatMap(s => [...document.querySelectorAll(s)])
                           .filter(e => e.innerText.trim().length > 80);
// Prefer the first matching editor selector; fall back only to manual review.
const editor = candidates[0];
const base = editor || document.body;
const links = [...document.querySelectorAll('a[href]')].map(a => a.href);
return {title: document.title, text: base.innerText, links,
        editor_found: !!editor, url: location.href};
"""

# Feishu's virtualized sidebar uses div nodes rather than ordinary hyperlinks.
# Read the public wiki token from DOM attributes, and click only expand arrows.
SIDEBAR_STEP = r"""
const root = document.querySelector('.wiki-space-detail-directory-tree');
if (!root) return {found: false, links: [], expanded: false, moved: false};
const nodes = [...root.querySelectorAll('[data-node-uid]')];
const links = nodes.map(e => {
  const token = new URLSearchParams(e.getAttribute('data-node-uid')).get('wikiToken');
  return token && /^[A-Za-z0-9]+$/.test(token) ? location.origin + '/wiki/' + token : null;
}).filter(Boolean);
const arrow = root.querySelector(
  '.workspace-tree-view-node-expand-arrow--collapsed.workspace-tree-view-node-expand-arrow--has-icon');
if (arrow) {
  arrow.scrollIntoView({block: 'nearest'});
  arrow.click();
  return {found: true, links, expanded: true, moved: false};
}
const area = root.closest('.workspace-scroll-area') ||
  document.querySelector('.wiki-sidebar-wrap .workspace-scroll-area');
let moved = false;
if (area) {
  const before = area.scrollTop;
  area.scrollTop += Math.max(100, area.clientHeight * .65);
  moved = area.scrollTop > before;
}
return {found: true, links, expanded: false, moved};
"""

SCROLL = r"""
const elements = [document.scrollingElement,
  ...document.querySelectorAll('div, main, section')].filter(e => e &&
    !e.closest('.wiki-sidebar-wrap, .workspace-next-sidebar-wrapper') &&
    e.clientHeight > 100 && e.scrollHeight > e.clientHeight + 100 &&
    (e === document.scrollingElement ||
     ['auto', 'scroll'].includes(getComputedStyle(e).overflowY)));
let moved = false;
for (const e of [...new Set(elements)]) {
  const before = e.scrollTop;
  e.scrollTop = Math.min(e.scrollHeight, before + Math.max(150, e.clientHeight * .7));
  if (e.scrollTop > before) moved = true;
}
return moved;
"""


def normalize(url, host):
    parts = urlsplit(url)
    if parts.scheme != 'https' or parts.netloc != host:
        return None
    if not re.fullmatch(r'/wiki/[A-Za-z0-9]+/?', parts.path):
        return None
    return f'https://{host}{parts.path.rstrip("/")}'


def save_json(path, data):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(path)


def discover_sidebar(driver, max_steps=1500):
    """Expand directory branches and sweep the virtualized tree top to bottom."""
    driver.execute_script("""
      const area = document.querySelector('.wiki-sidebar-wrap .workspace-scroll-area');
      if (area) area.scrollTop = 0;
    """)
    links = set()
    idle = 0
    for step in range(max_steps):
        result = driver.execute_script(SIDEBAR_STEP)
        new_links = set(result['links']) - links
        links.update(result['links'])
        if step % 25 == 0:
            print(f'  扫描侧边栏：已发现 {len(links)} 个文档，步骤 {step + 1}', flush=True)
        if not result['found']:
            if step >= 20:
                print('  未找到侧边栏，请确认左侧知识库目录已显示。', flush=True)
                break
            time.sleep(.5)
            continue
        idle = idle + 1 if not (new_links or result['expanded'] or result['moved']) else 0
        if idle >= 5:
            break
        time.sleep(.35 if result['expanded'] else .2)
    else:
        print('  侧边栏扫描达到步骤上限，可能仍有目录未发现。', flush=True)
    print(f'侧边栏扫描完成：发现 {len(links)} 个文档。', flush=True)
    return links


def collect(driver, scroll_steps):
    """Collect overlapping text snapshots to retain virtualized page sections."""
    chunks = []
    previous = ''
    links = set()
    editor_found = False
    latest = {}
    for _ in range(scroll_steps + 1):
        latest = driver.execute_script(SNAPSHOT)
        links.update(latest['links'])
        editor_found |= latest['editor_found']
        current = latest['text'].strip()
        if current and current != previous:
            # Merge exact overlaps; retain both fragments if the DOM changed.
            if previous and current.startswith(previous):
                chunks[-1] += current[len(previous):]
            elif not previous or current not in previous:
                overlap = 0
                for size in range(min(len(previous), len(current)), 39, -1):
                    if previous[-size:] == current[:size]:
                        overlap = size
                        break
                chunks.append(current[overlap:])
            previous = current
        if not driver.execute_script(SCROLL):
            break
        time.sleep(.65)
    return latest, '\n\n'.join(chunks), links, editor_found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('url', nargs='?', default=DEFAULT_URL)
    parser.add_argument('--output', type=Path, default=ROOT / 'docs/agv_docs/seer_api')
    parser.add_argument('--profile', type=Path,
                        default=Path.home() / 'snap/firefox/common/seer-wiki-export-profile'
                        if Path('/snap/firefox/current').exists() else
                        Path.home() / '.local/share/seer-wiki-export/firefox')
    parser.add_argument('--geckodriver', type=Path, help='显式指定 geckodriver 实际二进制路径')
    parser.add_argument('--check-browser', action='store_true',
                        help='无界面验证浏览器启动和页面读取后退出，不访问飞书')
    parser.add_argument('--max-pages', type=int, default=1000)
    parser.add_argument('--refresh', action='store_true', help='重新采集已保存页面')
    parser.add_argument('--scroll-steps', type=int, default=80)
    parser.add_argument('--delay', type=float, default=3)
    parser.add_argument('--seed-file', type=Path, help='额外入口链接，每行一个')
    args = parser.parse_args()
    host = urlsplit(args.url).netloc
    start = normalize(args.url, host)
    if not start:
        parser.error('请提供 https://域名/wiki/文档ID 格式的链接')
    if args.max_pages < 1 or args.scroll_steps < 1 or args.delay < 1:
        parser.error('max-pages、scroll-steps 至少为 1；delay 至少为 1 秒')
    try:
        from selenium import webdriver
        from selenium.webdriver.firefox.options import Options
        from selenium.webdriver.firefox.service import Service
        from selenium.common.exceptions import TimeoutException
    except ImportError as exc:
        parser.exit(1, f'Selenium 或其依赖导入失败：{exc}\n'
                    f'Python：{sys.executable}\n'
                    '请根据上面的实际错误检查依赖；安装步骤见使用说明。\n')

    args.output = args.output.expanduser().resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    args.profile = args.profile.expanduser().resolve()
    args.profile.mkdir(parents=True, exist_ok=True)
    args.profile.chmod(0o700)
    state_file = args.output / 'progress.json'
    state = json.loads(state_file.read_text(encoding='utf-8')) if state_file.exists() else {
        'pending': [], 'done': {}, 'failed': {}
    }
    if args.refresh:
        for url in state['done']:
            if url not in state['pending']:
                state['pending'].append(url)
    seeds = [start]
    if args.seed_file:
        seeds += args.seed_file.read_text(encoding='utf-8').splitlines()
    for seed in seeds:
        seed = normalize(seed.strip(), host)
        if seed and seed not in state['done'] and seed not in state['pending']:
            state['pending'].append(seed)
    # Retry failures on the next invocation.
    for url in list(state['failed']):
        if normalize(url, host) and url not in state['pending'] and url not in state['done']:
            state['pending'].append(url)
    options = Options()
    options.add_argument('-profile')
    options.add_argument(str(args.profile))
    # Keep the local WebDriver HTTP connection out of environment proxies.
    options.ignore_local_proxy_environment_variables()
    if args.check_browser:
        options.add_argument('-headless')
    snap_driver = Path('/snap/firefox/current/usr/lib/firefox/geckodriver')
    driver_path = (str(args.geckodriver.expanduser().resolve()) if args.geckodriver else
                   str(snap_driver) if snap_driver.is_file() else shutil.which('geckodriver'))
    if not driver_path:
        parser.exit(1, '未找到本地 geckodriver，请安装或通过 --geckodriver 指定路径。\n')
    log_file = args.output / 'geckodriver.log'
    service = Service(executable_path=driver_path, log_output=str(log_file))
    print(f'驱动：{driver_path}\n独立配置目录：{args.profile}', flush=True)
    try:
        driver = webdriver.Firefox(service=service, options=options)
    except Exception as exc:
        service.stop()
        parser.exit(1, f'Firefox 启动失败：{exc}\n驱动日志：{log_file}\n')
    if args.check_browser:
        try:
            driver.get('data:text/html,<title>seer-driver-check</title><main>OK</main>')
            assert driver.title == 'seer-driver-check', '浏览器页面读取验证失败'
            print('Firefox 启动、页面读取验证通过。')
        finally:
            driver.quit()
        return
    driver.set_page_load_timeout(60)
    try:
        driver.get(start)
        print('\n请在新打开的 Firefox 中登录飞书，回到目标知识库，保持左侧目录显示。')
        input('正文显示正常后，在本终端按回车开始（Ctrl+C 停止并保留进度）：')
        discovered = discover_sidebar(driver)
        save_json(args.output / 'sidebar_links.json', sorted(discovered))
        for link in sorted(discovered | set(driver.execute_script(SNAPSHOT)['links'])):
            link = normalize(urljoin(start, link), host)
            if link and link not in state['pending'] and link not in state['done']:
                state['pending'].append(link)
        save_json(state_file, state)
        count = 0
        attempted = set()
        while state['pending'] and count < args.max_pages:
            url = state['pending'][0]
            attempted.add(url)
            count += 1
            print(f'[{count}/{args.max_pages}] {url}', flush=True)
            try:
                try:
                    driver.get(url)
                except TimeoutException:
                    driver.execute_script('window.stop()')
                time.sleep(args.delay)
                # Wait for the rendered editor, including SPA loading.
                previous_signature = None
                stable = 0
                for _ in range(40):
                    snap = driver.execute_script(SNAPSHOT)
                    if urlsplit(snap['url']).netloc != host:
                        break
                    signature = (snap['title'], snap['text'][:500])
                    stable = stable + 1 if snap['editor_found'] and signature == previous_signature else 0
                    previous_signature = signature
                    if stable >= 4:
                        break
                    time.sleep(.5)
                if urlsplit(snap['url']).netloc != host:
                    print('页面跳到登录或其他站点，已停止。重新运行并登录即可续传。')
                    break
                snap, content, links, editor_found = collect(driver, args.scroll_steps)
                if not editor_found or len(content) < 100:
                    raise RuntimeError('未识别到文档正文，可能是权限、加载或页面结构问题')
                title = snap['title'].strip() or url.rsplit('/', 1)[-1]
                safe_title = re.sub(r'[^\w\u4e00-\u9fff.-]+', '_', title)[:90]
                suffix = hashlib.sha256(url.encode()).hexdigest()[:10]
                filename = f'{safe_title}_{suffix}.md'
                timestamp = datetime.now(timezone.utc).isoformat()
                (args.output / filename).write_text(
                    f'# {title}\n\n来源：{url}\n\n采集时间：{timestamp}\n\n'
                    '> 以下为浏览器可见文本。表格、图片和代码格式需对照原文复核。\n\n'
                    + content + '\n', encoding='utf-8')
                state['done'][url] = {'title': title, 'file': filename}
                state['failed'].pop(url, None)
                for link in sorted(links):
                    link = normalize(urljoin(url, link), host)
                    if (link and link not in state['done'] and link not in state['pending']
                            and link not in attempted):
                        state['pending'].append(link)
                print(f'  已保存：{filename}', flush=True)
            except Exception as exc:
                state['failed'][url] = str(exc)
                print(f'  失败，已记录：{exc}', flush=True)
            state['pending'].remove(url)
            save_json(state_file, state)
    except KeyboardInterrupt:
        print('\n已中断，保留待采集页面。')
    finally:
        save_json(state_file, state)
        lines = ['# 仙工 API 采集索引', '',
                 '仅包含成功读取的页面，不代表整个知识库已完整导出。', '']
        for url, item in state['done'].items():
            lines.append(f'- [{item["title"]}]({item["file"]}) · [原文]({url})')
        (args.output / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
        print(f'累计保存 {len(state["done"])} 页；待处理 {len(state["pending"])} 页；'
              f'失败 {len(state["failed"])} 页。输出：{args.output}')
        driver.quit()


if __name__ == '__main__':
    main()
