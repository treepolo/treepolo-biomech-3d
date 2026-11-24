/* 驗證指標（在頁面中執行）：骨架連貫性、擬合誤差、網格拉伸、陷入骨頭、肩胛—胸廓、效能。
   網格以 deformCPU 計算（與 GPU 著色器同一演算法）。 */
window.runMetrics=function(opt){opt=opt||{};const P=st.P,N=P.N,E=P.E,NB=MAN2.NB,T=P.ctx.T,out={id:P.id};
  const q=(a,p)=>{const s=Float64Array.from(a).sort();return s.length?s[Math.min(s.length-1,Math.floor(p*(s.length-1)))]:0};
  // ---------- 骨架 ----------
  const pairs=[];for(const S of['R','L'])pairs.push(['pelvis',S+'_femur'],[S+'_femur',S+'_tibia'],[S+'_tibia',S+'_foot'],['thorax',S+'_clav'],[S+'_scap',S+'_humerus'],[S+'_humerus',S+'_ulna'],[S+'_ulna',S+'_radius'],[S+'_radius',S+'_hand']);
  let gap=0,acGap=0,clavDev=0;const lenDev={};const bl=(i,a,b)=>V.len(V.sub(P.poses[i][b].O,P.poses[i][a].O));
  const L0={};for(const[a,b]of pairs)L0[a+'>'+b]=bl(0,a,b);
  for(let i=0;i<N;i++){const ps=P.poses[i];
    for(const[a,b]of pairs){gap=Math.max(gap,V.len(V.sub(ps[b].O,toWorld(ps[a],bindLoc(a,BIND[b].Ob)))));const k=a+'>'+b;lenDev[k]=Math.max(lenDev[k]||0,Math.abs(bl(i,a,b)-L0[k]))}
    for(const S of['R','L']){const AC=MAN.meta[S].AC;acGap=Math.max(acGap,V.len(V.sub(toWorld(ps[S+'_clav'],bindLoc(S+'_clav',AC)),toWorld(ps[S+'_scap'],bindLoc(S+'_scap',AC)))));
      clavDev=Math.max(clavDev,Math.abs(V.len(V.sub(toWorld(ps[S+'_clav'],bindLoc(S+'_clav',AC)),ps[S+'_clav'].O))/(P.ctx.sc.st*MAN.meta[S].lenC)-1))}}
  out.skeleton={jointGap_mm:+(gap*1000).toFixed(3),acGap_mm:+(acGap*1000).toFixed(3),clavLenDev_pct:+(clavDev*100).toFixed(3),boneLenDev_mm:+(Math.max(...Object.values(lenDev))*1000).toFixed(3),
    st:+P.ctx.sc.st.toFixed(3),comp:Object.fromEntries(Object.entries(P.ctx.sc.comp).map(([k,v])=>[k,+v.toFixed(3)])),flags:P.ctx.sc.flags,
    long:Object.fromEntries(['R','L'].map(S=>[S,Object.fromEntries(['F','T','H','A'].map(k=>[k,+P.ctx.sc[S][k].toFixed(3)]))])),
    res:Object.fromEntries(Object.entries(P.res).map(([k,v])=>[k,[+v.mean.toFixed(1),+v.p95.toFixed(1),+v.max.toFixed(1)]]))};
  // 肩胛取樣點比靜止時更深入胸廓（mm）
  {let worst=0;const th=BIND.thorax.i;for(let i=0;i<N;i++){const ps=P.poses[i];for(const S of['R','L'])for(const p of SCAPPTS[S]){const s=sdfCPU(th,toMesh(ps.thorax,'thorax',toWorld(ps[S+'_scap'],p.loc)));worst=Math.max(worst,Math.min(p.rest,.004)-s)}}out.skeleton.scapIntoThorax_mm=+(worst*1000).toFixed(1)}
  if(opt.skelOnly)return out;
  // ---------- 網格 ----------
  const frames=Object.assign({rest:-1},{stand0:0,PKH:E.pkh,FP:E.fp100,MER:E.mer,BR:E.br,MIR:E.mir});
  const segIds=MAN.segs.map((s,i)=>i);
  const sd=(si,p)=>{const T2=boneTris(si),{P:BP,fi,G,CS}=T2;let best=1e9,bt=-1,bx=0,by=0,bz=0;const cx=Math.floor(p[0]/CS),cy=Math.floor(p[1]/CS),cz=Math.floor(p[2]/CS);
    for(let a=-1;a<=1;a++)for(let b=-1;b<=1;b++)for(let c=-1;c<=1;c++){const l=G.get(((cx+a+512)*1024+(cy+b+512))*1024+(cz+c+512));if(!l)continue;for(const t of l){closestTriI(p[0],p[1],p[2],BP,fi[t*3],fi[t*3+1],fi[t*3+2]);const d=Math.hypot(_cp[0]-p[0],_cp[1]-p[1],_cp[2]-p[2]);if(d<best){best=d;bt=t;bx=_cp[0];by=_cp[1];bz=_cp[2]}}}
    if(bt<0)return null;const i0=fi[bt*3],i1=fi[bt*3+1],i2=fi[bt*3+2];const ux=BP[i1*3]-BP[i0*3],uy=BP[i1*3+1]-BP[i0*3+1],uz=BP[i1*3+2]-BP[i0*3+2],vx=BP[i2*3]-BP[i0*3],vy=BP[i2*3+1]-BP[i0*3+1],vz=BP[i2*3+2]-BP[i0*3+2];
    const nx=uy*vz-uz*vy,ny=uz*vx-ux*vz,nz=ux*vy-uy*vx;const s=(p[0]-bx)*nx+(p[1]-by)*ny+(p[2]-bz)*nz;return s<0?-best:best};
  const edges=mu=>{if(mu._E)return mu._E;const s=new Set(),E2=[];const f=mu.faces;for(let t=0;t<mu.nf;t++)for(const[a,b]of[[f[t*3],f[t*3+1]],[f[t*3+1],f[t*3+2]],[f[t*3+2],f[t*3]]]){const k=a<b?a*65536+b:b*65536+a;if(!s.has(k)){s.add(k);E2.push(a,b)}}return mu._E=E2};
  const ln=(p,a,b)=>Math.hypot(p[a*3]-p[b*3],p[a*3+1]-p[b*3+1],p[a*3+2]-p[b*3+2]);
  const useField=opt.field!==false;const posAt=(fi)=>{const D=new Float32Array(FIB.length*MP*12);frameFibers(P,fi,D);const Fd=useField?fieldFrame(P,fi,D,new Float32Array(FLD.data.length)):null;const o={};
    for(const t of TIS){if(t.kind!=='m')continue;const p=new Float32Array(t.mu.nv*3);deformCPU(t.mu,P,fi,D,Fd,opt.push!==false,p);o[t.id]=p}return o};
  const REST=posAt(-1);
  const vnorm=(p,mu)=>{const n=new Float32Array(mu.nv*3),f=mu.faces;for(let t=0;t<mu.nf;t++){const a=f[t*3],b=f[t*3+1],c=f[t*3+2];const ux=p[b*3]-p[a*3],uy=p[b*3+1]-p[a*3+1],uz=p[b*3+2]-p[a*3+2],vx=p[c*3]-p[a*3],vy=p[c*3+1]-p[a*3+1],vz=p[c*3+2]-p[a*3+2];const x=uy*vz-uz*vy,y=uz*vx-ux*vz,z=ux*vy-uy*vx;for(const k of[a,b,c]){n[k*3]+=x;n[k*3+1]+=y;n[k*3+2]+=z}}
    for(let v=0;v<mu.nv;v++){const l=Math.hypot(n[v*3],n[v*3+1],n[v*3+2])||1;n[v*3]/=l;n[v*3+1]/=l;n[v*3+2]/=l}return n};
  // 肌肉—肌肉穿越：靜止時相距 < 6 mm 的不同肌肉頂點對，記錄 B 在 A 法向上的正負；該幀正負翻轉 = 穿過
  const PAIRS=[];{const H=new Map(),cs=.006,key=(x,y,z)=>((Math.floor(x/cs)+4096)*8192+(Math.floor(y/cs)+4096))*8192+(Math.floor(z/cs)+4096);const ids=TIS.filter(t=>t.kind==='m').map(t=>t.id);
    ids.forEach((id,mi)=>{const p=REST[id];for(let v=0;v<MUS[id].nv;v+=2){const k=key(p[v*3],p[v*3+1],p[v*3+2]);let l=H.get(k);if(!l){l=[];H.set(k,l)}l.push(mi,v)}});
    ids.forEach((id,mi)=>{const p=REST[id],n=vnorm(p,MUS[id]);for(let v=0;v<MUS[id].nv;v+=4){const x=p[v*3],y=p[v*3+1],z=p[v*3+2];let best=.006,bm=-1,bv=-1;
      for(let a=-1;a<=1;a++)for(let b=-1;b<=1;b++)for(let c=-1;c<=1;c++){const l=H.get(key(x+a*cs,y+b*cs,z+c*cs));if(!l)continue;for(let j=0;j<l.length;j+=2){if(l[j]===mi)continue;const q=REST[ids[l[j]]],w=l[j+1];const d=Math.hypot(q[w*3]-x,q[w*3+1]-y,q[w*3+2]-z);if(d<best){best=d;bm=l[j];bv=w}}}
      if(bm<0)continue;const q=REST[ids[bm]];const s0=(q[bv*3]-x)*n[v*3]+(q[bv*3+1]-y)*n[v*3+1]+(q[bv*3+2]-z)*n[v*3+2];if(Math.abs(s0)>.0005)PAIRS.push(id,v,ids[bm],bv,s0)}})}
  out.crossPairs=PAIRS.length/5;
  out.mesh={};const top={};
  for(const[fn,fi]of Object.entries(frames)){const A=fi<0?P.AR:P.mats[fi];const PP=fi<0?REST:posAt(fi);let tot=0,bad=0,mw=0,chk=0,in3=0,in10=0,badV=0;const per=[];
    for(const t of TIS){if(t.kind!=='m')continue;const mu=t.mu,R={P:REST[t.id]};const p=PP[t.id];
      const E2=edges(mu),rr=[];for(let e=0;e<E2.length;e+=2){const l0=ln(R.P,E2[e],E2[e+1]);if(l0<1e-5)continue;rr.push(ln(p,E2[e],E2[e+1])/l0)}
      const med=q(rr,.5);let b=0,mx=0,bb=0;for(let j=0,e=0;e<E2.length;e+=2){const l0=ln(R.P,E2[e],E2[e+1]);if(l0<1e-5)continue;const r=rr[j++],z=r/med;if(z>2.5||z<.4){b++;if(Math.abs(r-med)*l0>.002)bb++}mx=Math.max(mx,z)}badV+=bb;tot+=rr.length;bad+=b;if(b)mw++;per.push([t.name,b,rr.length,+mx.toFixed(2)]);
      if(!opt.noPen)for(let v=0;v<mu.nv;v+=3){const w=[p[v*3],p[v*3+1],p[v*3+2]];let worst=1e9;
        for(const si of segIds){const o=si*12;const Ai=fi<0?P.ARi:P.matsI[fi];const bpt=affP(Ai,si,w);const T2=boneTris(si);if(bpt[0]<T2.bmn[0]-.03||bpt[0]>T2.bmx[0]+.03||bpt[1]<T2.bmn[1]-.03||bpt[1]>T2.bmx[1]+.03||bpt[2]<T2.bmn[2]-.03||bpt[2]>T2.bmx[2]+.03)continue;
          const sc=Math.cbrt(Math.abs(A[o]*(A[o+4]*A[o+8]-A[o+5]*A[o+7])-A[o+1]*(A[o+3]*A[o+8]-A[o+5]*A[o+6])+A[o+2]*(A[o+3]*A[o+7]-A[o+4]*A[o+6])));const d=sd(si,bpt);if(d!==null&&d*sc<worst)worst=d*sc}
        chk++;if(worst<-.003)in3++;if(worst<-.010)in10++}}
    {const nc={};let cr=0;for(let j=0;j<PAIRS.length;j+=5){const A_=PAIRS[j],v=PAIRS[j+1],B_=PAIRS[j+2],w=PAIRS[j+3],s0=PAIRS[j+4];const n=nc[A_]||(nc[A_]=vnorm(PP[A_],MUS[A_]));const p=PP[A_],q=PP[B_];
      const s1=(q[w*3]-p[v*3])*n[v*3]+(q[w*3+1]-p[v*3+1])*n[v*3+1]+(q[w*3+2]-p[v*3+2])*n[v*3+2];if(s0*s1<0&&Math.abs(s1)>.001){cr++;const k=MUS[A_].h.id;(out._cr=out._cr||{})[fn]=out._cr[fn]||{};out._cr[fn][k]=(out._cr[fn][k]||0)+1}}var crossPct=+(cr/(PAIRS.length/5)*100).toFixed(2)}
    per.sort((a,b)=>b[1]/b[2]-a[1]/a[2]);top[fn]=per.slice(0,8);
    out.mesh[fn]={musCrossPct:crossPct,badEdgePct:+(bad/tot*100).toFixed(3),visBadPct:+(badV/tot*100).toFixed(3),badEdges:bad,musclesWithBad:mw,penSampled:chk,inside3mmPct:chk?+(in3/chk*100).toFixed(2):null,inside10mm:in10}}
  out.meshTop=top;if(out._cr){out.crossTop={};for(const fn in out._cr){out.crossTop[fn]=Object.entries(out._cr[fn]).sort((a,b)=>b[1]-a[1]).slice(0,15).map(([id,c])=>[TIS.find(t=>t.id===id).name,c])}delete out._cr}
  // ---------- 效能 ----------
  {const t0=performance.now();for(let k=0;k<20;k++){st.f=(k*17)%N;updateScene()}out.perf={updateScene_ms:+((performance.now()-t0)/20).toFixed(2)}}
  return out};
/* 力臂檢查：在個人靜止姿勢下把遠端骨群繞關節軸轉 ±5°，路徑長的變化 dL/dθ（cm/rad；負 = 該動作使肌肉縮短 = 該動作的作用肌） */
window.momentArms=function(){const P=st.P,NB=MAN2.NB,n=MAN.segs.length,id=nm=>BIND[nm].i;
  const rotSub=(bones,c,ax,ang)=>{const A=Float64Array.from(P.AR);const s=Math.sin(ang),co=Math.cos(ang),[x,y,z]=ax,t=1-co;const R=[[t*x*x+co,t*x*y-s*z,t*x*z+s*y],[t*x*y+s*z,t*y*y+co,t*y*z-s*x],[t*x*z-s*y,t*y*z+s*x,t*z*z+co]];
    for(const b of bones){const o=b*12;const L=[0,1,2].map(r=>[0,1,2].map(k=>R[r][0]*P.AR[o+k]+R[r][1]*P.AR[o+3+k]+R[r][2]*P.AR[o+6+k]));const tt=[0,1,2].map(r=>P.AR[o+9+r]-c[r]);
      for(let r=0;r<3;r++){for(let k=0;k<3;k++)A[o+r*3+k]=L[r][k];A[o+9+r]=c[r]+R[r][0]*tt[0]+R[r][1]*tt[1]+R[r][2]*tt[2]}}
    const Ai=new Float64Array(NB*12);for(let b=0;b<NB;b++)affInv(A,b,Ai);return[A,Ai]};
  const restJ=(S,k)=>affP(P.AR,id(S+'_'+{hip:'femur',knee:'tibia',ankle:'foot',sh:'humerus',el:'ulna'}[k]),BIND[S+'_'+{hip:'femur',knee:'tibia',ankle:'foot',sh:'humerus',el:'ulna'}[k]].Ob);
  const tests=[];const S='R',toe=n;
  const J={hip:[[id('R_femur'),id('R_tibia'),id('R_foot'),toe],restJ(S,'hip'),[0,0,1],'髖屈曲'],knee:[[id('R_tibia'),id('R_foot'),toe],restJ(S,'knee'),[0,0,-1],'膝屈曲'],ankle:[[id('R_foot'),toe],restJ(S,'ankle'),[0,0,1],'踝背屈'],
    el:[[id('R_ulna'),id('R_radius'),id('R_hand')],restJ(S,'el'),[0,0,1],'肘屈曲'],shAbd:[[id('R_humerus'),id('R_ulna'),id('R_radius'),id('R_hand')],restJ(S,'sh'),[-1,0,0],'肩外展'],
    shER:[[id('R_humerus'),id('R_ulna'),id('R_radius'),id('R_hand')],restJ(S,'sh'),[0,-1,0],'肩外旋']};
  const pick={hip:['iliacus','psoas major','rectus femoris','gluteus maximus','long head of biceps femoris','semimembranosus','adductor longus'],knee:['long head of biceps femoris','semitendinosus','vastus lateralis','vastus medialis','rectus femoris','medial head of gastrocnemius'],
    ankle:['soleus','medial head of gastrocnemius','tibialis anterior','fibularis longus'],el:['long head of biceps brachii','short head of biceps brachii','brachialis','brachioradialis','lateral head of triceps brachii','long head of triceps brachii'],
    shAbd:['acromial part of deltoid','supraspinatus','latissimus dorsi','teres major','sternocostal part of pectoralis major'],shER:['infraspinatus muscle','teres minor','subscapularis','sternocostal part of pectoralis major','latissimus dorsi']};
  const out={};for(const k in J){const[bones,c,ax,nm]=J[k];const[Ap,Aip]=rotSub(bones,c,ax,5*D2R),[Am,Aim]=rotSub(bones,c,ax,-5*D2R);out[nm]={};
    for(const base of pick[k]){const t=TIS.find(x=>x.key===base&&x.side==='R');if(!t){out[nm][base]='—';continue}const d=(pathLen(t.mu,Ap,Aip)-pathLen(t.mu,Am,Aim))/(10*D2R)*100;out[nm][t.name]=+d.toFixed(2)}}
  return out};
