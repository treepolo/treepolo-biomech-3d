"""自動驗證：以無頭 Chrome 開 index.html，對 20 顆內建球跑骨架與網格指標（tools/metrics.js），輸出 JSON。
用法：python3 tools/verify.py index.html out.json [--pen]（需要 playwright 與 Chrome）"""
import asyncio, json, sys, os
from playwright.async_api import async_playwright
here = os.path.dirname(os.path.abspath(__file__))
async def main(page, out, pen):
    lib = open(os.path.join(here, 'tri_helpers.js')).read() + '\n' + open(os.path.join(here, 'metrics.js')).read()
    async with async_playwright() as p:
        b = await p.chromium.launch(args=['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'])
        pg = await b.new_page()
        await pg.goto('file://' + os.path.abspath(page)); await pg.wait_for_function('window.__ready===true', timeout=300000)
        await pg.add_script_tag(content=lib)
        r = await pg.evaluate("""async(pen)=>{st.playing=false;const out=[];for(const p of PD.pitches){await pickPitch(p.id);const m=runMetrics({noPen:!pen});out.push({id:p.id,skeleton:m.skeleton,mesh:m.mesh,perf:m.perf})}
          await pickPitch(PD.pitches[0].id);return {pitches:out,momentArms:momentArms()}}""", pen)
        json.dump(r, open(out, 'w'), ensure_ascii=False, indent=1); await b.close()
asyncio.run(main(sys.argv[1], sys.argv[2], '--pen' in sys.argv))
