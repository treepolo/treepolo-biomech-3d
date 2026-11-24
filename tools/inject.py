"""把 build.py 產生的 man2.json、blob_mus2.txt、blob_sdf.txt 寫回 index.html（取代同名 script 區塊）。
用法：python3 tools/inject.py index.html <工作資料夾>"""
import re, sys, os
path, wd = sys.argv[1], sys.argv[2]; html = open(path, encoding='utf-8').read()
rep = {'man2': ('application/json', open(os.path.join(wd, 'man2.json')).read()), 'mus2': ('text/plain', open(os.path.join(wd, 'blob_mus2.txt')).read()), 'sdf': ('text/plain', open(os.path.join(wd, 'blob_sdf.txt')).read())}
for k, (t, v) in rep.items():
    html, n = re.subn(r'<script type="[\w/]+" id="%s">.*?</script>' % k, lambda m: '<script type="%s" id="%s">%s</script>' % (t, k, v), html, flags=re.S)
    assert n == 1, k
open(path, 'w', encoding='utf-8').write(html); print('ok')
