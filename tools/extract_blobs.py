"""把 index.html 內嵌的資料（man 骨骼/後設、pd 球員、bin 骨骼網格）取出到工作資料夾，供 build.py 使用。
用法：python3 tools/extract_blobs.py index.html <工作資料夾>"""
import re, sys, os, shutil
html = open(sys.argv[1], encoding='utf-8').read(); out = sys.argv[2]; os.makedirs(out, exist_ok=True)
for k in ('man', 'pd', 'bin'):
    m = re.search(r'<script type="[\w/]+" id="%s">(.*?)</script>' % k, html, re.S)
    open(os.path.join(out, 'blob_%s.txt' % k), 'w').write(m.group(1))
here = os.path.dirname(os.path.abspath(__file__))
shutil.copy(os.path.join(here, 'data', 'muscle_rest.json'), os.path.join(out, 'rest_export.json'))
shutil.copy(os.path.join(here, 'data', 'att_table.js'), os.path.join(out, 'att_table.js'))
print('ok')
