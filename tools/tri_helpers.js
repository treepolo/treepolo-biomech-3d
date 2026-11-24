/* 驗證用：骨面三角網格最近點（與舊版量測相同的方法，供前後比較） */
const _BT={};
function boneTris(si){if(_BT[si])return _BT[si];const sub=si>=100?{h:(si-100)>>1,z:(si-100)&1}:null;const bsi=sub?BIND[(sub.h?'L':'R')+'_foot'].i:si;const s=MAN.segs[bsi],CS=.03;const iv=new Int16Array(BIN,s.off,s.nv*3),fi=new Uint16Array(BIN,s.off+s.nv*6,s.nf*3);
  const P=new Float64Array(s.nv*3);for(let i=0;i<s.nv;i++){const lx=iv[i*3]*s.q,ly=iv[i*3+1]*s.q,lz=iv[i*3+2]*s.q;for(let c=0;c<3;c++)P[i*3+c]=s.Ob[c]+lx*s.Fb[0][c]+ly*s.Fb[1][c]+lz*s.Fb[2][c]}
  const G=new Map();const fox=sub?BIND[(sub.h?'L':'R')+'_foot'].Ob[0]:0;for(let t=0;t<s.nf;t++){const a=fi[t*3],b=fi[t*3+1],c=fi[t*3+2];if(sub){const lx=(P[a*3]+P[b*3]+P[c*3])/3-fox;const isz=lx>=MTPX-.005;if(isz!==!!sub.z)continue}let mn=[1e9,1e9,1e9],mx=[-1e9,-1e9,-1e9];for(const v of[a,b,c])for(let k=0;k<3;k++){mn[k]=Math.min(mn[k],P[v*3+k]);mx[k]=Math.max(mx[k],P[v*3+k])}
    for(let x=Math.floor(mn[0]/CS);x<=Math.floor(mx[0]/CS);x++)for(let y=Math.floor(mn[1]/CS);y<=Math.floor(mx[1]/CS);y++)for(let z=Math.floor(mn[2]/CS);z<=Math.floor(mx[2]/CS);z++){const k=((x+512)*1024+(y+512))*1024+(z+512);let l=G.get(k);if(!l){l=[];G.set(k,l)}l.push(t)}}
  let bmn=[1e9,1e9,1e9],bmx=[-1e9,-1e9,-1e9];for(let i=0;i<s.nv;i++)for(let c=0;c<3;c++){const x=P[i*3+c];if(x<bmn[c])bmn[c]=x;if(x>bmx[c])bmx[c]=x}
  const segOfTri=null;
  return _BT[si]={P,fi,G,CS,nf:s.nf,bmn,bmx}}
const _cp=[0,0,0];
function closestTriI(px,py,pz,P,i0,i1,i2){// Ericson, 無配置；結果寫入 _cp
  const ax=P[i0*3],ay=P[i0*3+1],az=P[i0*3+2],bx=P[i1*3],by=P[i1*3+1],bz=P[i1*3+2],cx=P[i2*3],cy=P[i2*3+1],cz=P[i2*3+2];
  const abx=bx-ax,aby=by-ay,abz=bz-az,acx=cx-ax,acy=cy-ay,acz=cz-az,apx=px-ax,apy=py-ay,apz=pz-az;
  const d1=abx*apx+aby*apy+abz*apz,d2=acx*apx+acy*apy+acz*apz;if(d1<=0&&d2<=0){_cp[0]=ax;_cp[1]=ay;_cp[2]=az;return}
  const bpx=px-bx,bpy=py-by,bpz=pz-bz,d3=abx*bpx+aby*bpy+abz*bpz,d4=acx*bpx+acy*bpy+acz*bpz;if(d3>=0&&d4<=d3){_cp[0]=bx;_cp[1]=by;_cp[2]=bz;return}
  const vc=d1*d4-d3*d2;if(vc<=0&&d1>=0&&d3<=0){const t=d1/(d1-d3);_cp[0]=ax+t*abx;_cp[1]=ay+t*aby;_cp[2]=az+t*abz;return}
  const cpx=px-cx,cpy=py-cy,cpz=pz-cz,d5=abx*cpx+aby*cpy+abz*cpz,d6=acx*cpx+acy*cpy+acz*cpz;if(d6>=0&&d5<=d6){_cp[0]=cx;_cp[1]=cy;_cp[2]=cz;return}
  const vb=d5*d2-d1*d6;if(vb<=0&&d2>=0&&d6<=0){const w=d2/(d2-d6);_cp[0]=ax+w*acx;_cp[1]=ay+w*acy;_cp[2]=az+w*acz;return}
  const va=d3*d6-d5*d4;if(va<=0&&(d4-d3)>=0&&(d5-d6)>=0){const w=(d4-d3)/((d4-d3)+(d5-d6));_cp[0]=bx+w*(cx-bx);_cp[1]=by+w*(cy-by);_cp[2]=bz+w*(cz-bz);return}
  const den=1/(va+vb+vc),vv=vb*den,ww=vc*den;_cp[0]=ax+abx*vv+acx*ww;_cp[1]=ay+aby*vv+acy*ww;_cp[2]=az+abz*vv+acz*ww}
