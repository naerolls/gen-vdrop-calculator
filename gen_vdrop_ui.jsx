/**
 * gen_vdrop_ui.jsx — Generator Voltage Drop Calculator (React UI)
 *
 * Features:
 *   - Salient-pole Newton-Raphson solver (JS port of gen_vdrop.py)
 *   - Step load + motor starting studies
 *   - Saturation correction toggle (SE_max / SE_75max)
 *   - Allen-Bradley CGCM AVR response toggle (IEEE Type I model)
 *   - Voltage recovery curve (SVG)
 *   - Auto-generated CLI command for gen_vdrop.py
 *
 * GitHub: https://github.com/naerolls/gen-vdrop-calculator
 */

import { useState, useCallback } from "react";

// ── NR solver ──────────────────────────────────────────────────────────────
function calcEqPre(Vt, Ib, phiB, Xd, Xq, Ra) {
  if (Ib < 1e-9) return { Eq: Vt, delta: 0 };
  let d = Math.atan2(Xq*Ib*Math.cos(phiB), Vt+Xq*Ib*Math.sin(phiB));
  for (let i=0;i<100;i++){
    const Id=Ib*Math.sin(d+phiB), Iq=Ib*Math.cos(d+phiB);
    const f2=Vt*Math.sin(d)-Xq*Iq+Ra*Id;
    const df2=Vt*Math.cos(d)-Xq*(-Ib*Math.sin(d+phiB))+Ra*(Ib*Math.cos(d+phiB));
    if(Math.abs(df2)<1e-15)break; const s=f2/df2; d-=s; if(Math.abs(s)<1e-12)break;
  }
  const Id=Ib*Math.sin(d+phiB), Iq=Ib*Math.cos(d+phiB);
  return {Eq:Vt*Math.cos(d)+Ra*Iq+Xd*Id, delta:d};
}
function calcEqSat(Vt,Ib,phiB,Xd,Xq,Ra,A,B){
  const{Eq:EqL,delta}=calcEqPre(Vt,Ib,phiB,Xd,Xq,Ra);
  if(A<=0||Ib<1e-9)return{Eq:EqL,delta};
  return{Eq:EqL+A*Math.exp(B*EqL)*Xd*Ib*Math.sin(delta+phiB),delta};
}
function satCoeffs(sm,s75){
  if(s75<=0||sm<=s75)return{A:0,B:0};
  const B=Math.log(sm/s75)/0.25; return{A:s75/Math.exp(0.75*B),B};
}
function NR(Eq,S,phi,Xd,Xq,Ra){
  let V=0.9,d=Math.PI/12;
  for(let i=0;i<200;i++){
    const I=S/V,Id=I*Math.sin(d+phi),Iq=I*Math.cos(d+phi);
    const f1=Eq-V*Math.cos(d)-Ra*Iq-Xd*Id, f2=V*Math.sin(d)-Xq*Iq+Ra*Id;
    if(Math.abs(f1)<1e-9&&Math.abs(f2)<1e-9)return V;
    const dIdV=-I/V;
    const J00=-Math.cos(d)-Ra*dIdV*Math.cos(d+phi)-Xd*dIdV*Math.sin(d+phi);
    const J01=V*Math.sin(d)-Ra*(-I*Math.sin(d+phi))-Xd*(I*Math.cos(d+phi));
    const J10=Math.sin(d)-Xq*dIdV*Math.cos(d+phi)+Ra*dIdV*Math.sin(d+phi);
    const J11=V*Math.cos(d)-Xq*(-I*Math.sin(d+phi))+Ra*(I*Math.cos(d+phi));
    const det=J00*J11-J01*J10;
    if(Math.abs(det)<1e-15)return null;
    V=Math.max(0.005,V-(J11*f1-J01*f2)/det); d-=(-J10*f1+J00*f2)/det;
  }
  return null;
}
function simpDip(X,I,pf){
  const disc=1-(X*I*pf)**2;
  return disc<0?null:Math.sqrt(disc)-X*I*Math.sqrt(Math.max(0,1-pf*pf));
}

// ── IEEE Type I AVR simulation ─────────────────────────────────────────────
/**
 * Two-phase simulation:
 *   Phase 1 (0–50 ms):   Subtransient — Xd"/Xq" fixed, AVR has not responded.
 *   Phase 2 (50 ms–end): Transient+recovery — Xd'/Xq', AVR active.
 *
 * State equations (Euler):
 *   dVR/dt = (KA*(Vref-Vt) - VR) / TA
 *   dEq/dt = (VR - KE*Eq) / TE
 */
function simulateAVR(gen, avr, Spu, phi, Ib=0, phiB=0){
  const dt=0.002, Tsub=0.05, Ttot=2.5;
  const Eq_pp=calcEqPre(1,Ib,phiB,gen.Xdpp,gen.Xqpp,gen.Ra).Eq;
  const Eq_p =calcEqPre(1,Ib,phiB,gen.Xdp, gen.Xqp, gen.Ra).Eq;
  const VR0=avr.KE*Eq_p, Vref=1+VR0/avr.KA;
  const times=[], volts=[];
  for(let t=0;t<Tsub-dt/2;t+=dt){
    const Vt=NR(Eq_pp,Spu,phi,gen.Xdpp,gen.Xqpp,gen.Ra)||0;
    times.push(parseFloat(t.toFixed(4))); volts.push(Vt);
  }
  let Eq=Eq_p, VR=VR0;
  for(let t=Tsub;t<=Ttot+dt/2;t+=dt){
    const Vt=NR(Eq,Spu,phi,gen.Xdp,gen.Xqp,gen.Ra)||0;
    times.push(parseFloat(t.toFixed(4))); volts.push(Vt);
    const dVR=(avr.KA*(Vref-Vt)-VR)/avr.TA, dEq=(VR-avr.KE*Eq)/avr.TE;
    VR=Math.max(avr.VRMIN,Math.min(avr.VRMAX,VR+dVR*dt)); Eq+=dEq*dt;
  }
  return{times,volts};
}

// ── Study runner ───────────────────────────────────────────────────────────
function runStudy(gen,mode,stepLoad,motorParams,satOn,sA,sB){
  const periods=[
    {label:"Subtransient",sub:"0 – 50 ms",cls:"sub",Xd:gen.Xdpp,Xq:gen.Xqpp},
    {label:"Transient",sub:"50 – 500 ms",cls:"trn",Xd:gen.Xdp,Xq:gen.Xqp},
    {label:"Steady-state",sub:"> 1 s (open-loop)",cls:"ss",Xd:gen.Xd,Xq:gen.Xq},
  ];
  let Spu,phi,Ib=0,phiB=0;
  if(mode==="step"){
    Spu=(stepLoad.kw/stepLoad.pf)/gen.kva; phi=Math.acos(stepLoad.pf);
  } else {
    const bK=motorParams.baseKva||0,bP=motorParams.basePf||0.85;
    const mK=motorParams.motorKva,mP=motorParams.motorPf;
    const P=bK*bP+mK*mP,Q=bK*Math.sqrt(Math.max(0,1-bP**2))+mK*Math.sqrt(Math.max(0,1-mP**2));
    Spu=Math.sqrt(P**2+Q**2)/gen.kva; phi=Math.atan2(Q,P);
    Ib=bK/gen.kva; phiB=Math.acos(bP);
  }
  const results=periods.map(p=>{
    if(!p.Xd||!p.Xq)return{...p,Eq:null,Vt:null,Vsimp:null};
    const EqRes=(mode==="motor"&&satOn&&sA>0)
      ?calcEqSat(1,Ib,phiB,p.Xd,p.Xq,gen.Ra,sA,sB)
      :calcEqPre(1,Ib,phiB,p.Xd,p.Xq,gen.Ra);
    return{...p,Eq:EqRes.Eq,Vt:NR(EqRes.Eq,Spu,phi,p.Xd,p.Xq,gen.Ra),Vsimp:simpDip(p.Xd,Spu,Math.cos(phi))};
  });
  return{results,Spu,phi,Ib,phiB};
}

// ── SVG chart ──────────────────────────────────────────────────────────────
function VoltageChart({times,volts,olPP,olP,minV,genVolt}){
  if(!times||!times.length)return null;
  const W=420,H=180,PL=36,PR=12,PT=12,PB=26;
  const cW=W-PL-PR,cH=H-PT-PB,yMin=0.55,yMax=1.18,tMax=times[times.length-1];
  const tx=t=>PL+cW*(t/tMax), ty=v=>PT+cH*(1-(v-yMin)/(yMax-yMin));
  let path="";
  times.forEach((t,i)=>{const x=tx(t).toFixed(1),y=ty(volts[i]).toFixed(1);path+=i===0?`M${x},${y}`:`L${x},${y}`;});
  const recT=times.find((t,i)=>volts[i]>=0.95&&t>0.06);
  const vSS=volts[volts.length-1];
  return(<div>
    <svg width="100%" viewBox={`0 0 ${W} ${H}`} style={{display:"block"}}>
      <rect x={PL} y={PT} width={cW} height={cH} fill="none" stroke="var(--color-border-tertiary)" strokeWidth="0.5"/>
      {[0.6,0.7,0.8,0.9,1.0,1.1].map(yv=><g key={yv}>
        <line x1={PL} y1={ty(yv).toFixed(1)} x2={W-PR} y2={ty(yv).toFixed(1)} stroke="var(--color-border-tertiary)" strokeWidth="0.5"/>
        <text x={PL-3} y={(ty(yv)+3.5).toFixed(1)} textAnchor="end" fontSize="8" fill="var(--color-text-secondary)">{yv.toFixed(1)}</text>
      </g>)}
      {[0,0.5,1,1.5,2,2.5].filter(t=>t<=tMax).map(xt=><g key={xt}>
        <line x1={tx(xt).toFixed(1)} y1={PT} x2={tx(xt).toFixed(1)} y2={H-PB} stroke="var(--color-border-tertiary)" strokeWidth="0.5"/>
        <text x={tx(xt).toFixed(1)} y={H-PB+10} textAnchor="middle" fontSize="8" fill="var(--color-text-secondary)">{xt}s</text>
      </g>)}
      <line x1={tx(0.05).toFixed(1)} y1={PT} x2={tx(0.05).toFixed(1)} y2={H-PB} stroke="#9fe1cb" strokeWidth="1" strokeDasharray="3,2"/>
      <text x={(tx(0.05)+2).toFixed(1)} y={PT+9} fontSize="8" fill="#0f6e56">50ms</text>
      <line x1={tx(0.5).toFixed(1)} y1={PT} x2={tx(0.5).toFixed(1)} y2={H-PB} stroke="#fac775" strokeWidth="1" strokeDasharray="3,2"/>
      <text x={(tx(0.5)+2).toFixed(1)} y={PT+9} fontSize="8" fill="#854f0b">500ms</text>
      {olPP&&<line x1={PL} y1={ty(olPP).toFixed(1)} x2={W-PR} y2={ty(olPP).toFixed(1)} stroke="#5dcaa5" strokeWidth="1" strokeDasharray="4,3" opacity="0.7"/>}
      {olP &&<line x1={PL} y1={ty(olP).toFixed(1)}  x2={W-PR} y2={ty(olP).toFixed(1)}  stroke="#ef9f27" strokeWidth="1" strokeDasharray="4,3" opacity="0.7"/>}
      <line x1={PL} y1={ty(minV).toFixed(1)} x2={W-PR} y2={ty(minV).toFixed(1)} stroke="#e24b4a" strokeWidth="1" strokeDasharray="2,2" opacity="0.8"/>
      <text x={PL+2} y={(ty(minV)-2).toFixed(1)} fontSize="8" fill="#a32d2d">min {minV.toFixed(2)} pu</text>
      <line x1={PL} y1={ty(1).toFixed(1)} x2={W-PR} y2={ty(1).toFixed(1)} stroke="#1d9e75" strokeWidth="0.8" opacity="0.35"/>
      <path d={path} fill="none" stroke="#185fa5" strokeWidth="2" strokeLinejoin="round"/>
      <text x={PL-1} y={PT-2} fontSize="8" fill="var(--color-text-secondary)">pu</text>
    </svg>
    <div style={{display:"grid",gridTemplateColumns:"1fr 1fr 1fr",gap:6,marginTop:8}}>
      {[
        {label:"Peak dip (0–50 ms)",val:`${((1-volts[0])*100).toFixed(1)}%`,sub:"AVR cannot help — same as open-loop"},
        {label:"Recovery to 0.95 pu",val:recT?`~${recT.toFixed(2)} s`:"> 2.5 s",sub:"from AVR boost"},
        {label:`Steady-state at ${tMax.toFixed(1)} s`,val:`${(vSS*100).toFixed(1)}%`,sub:`${vSS.toFixed(3)} pu — AVR regulated`},
      ].map(m=><div key={m.label} style={{background:"var(--color-background-secondary)",borderRadius:"var(--border-radius-md)",padding:"7px 9px"}}>
        <div style={{fontSize:9,color:"var(--color-text-secondary)",textTransform:"uppercase",letterSpacing:".06em"}}>{m.label}</div>
        <div style={{fontSize:16,fontWeight:500,color:"var(--color-text-primary)",marginTop:2}}>{m.val}</div>
        <div style={{fontSize:9,color:"var(--color-text-secondary)"}}>{m.sub}</div>
      </div>)}
    </div>
  </div>);
}

// ── UI helpers ─────────────────────────────────────────────────────────────
const Field=({label,hint,children})=><div style={{marginBottom:6}}>
  <label style={{display:"block",fontSize:10,color:"var(--color-text-secondary)",marginBottom:2}}>{label}</label>
  {children}
  {hint&&<div style={{fontSize:9,color:"var(--color-text-tertiary)",marginTop:1}}>{hint}</div>}
</div>;

const NumIn=({value,onChange,step="any"})=><input type="number" value={value} step={step}
  onChange={e=>onChange(parseFloat(e.target.value)||0)}
  style={{width:"100%",height:28,padding:"0 7px",fontSize:12,fontFamily:"var(--font-mono)",color:"var(--color-text-primary)",
    background:"var(--color-background-primary)",border:"0.5px solid var(--color-border-secondary)",borderRadius:"var(--border-radius-md)"}}/>;

const Tog=({on,onToggle,label,sub})=><div onClick={onToggle} style={{display:"flex",alignItems:"center",gap:8,padding:"6px 8px",
  border:"0.5px solid var(--color-border-tertiary)",borderRadius:"var(--border-radius-md)",
  background:"var(--color-background-primary)",cursor:"pointer",marginBottom:6}}>
  <div style={{width:30,height:17,borderRadius:9,background:on?"#1d9e75":"var(--color-border-secondary)",position:"relative",flexShrink:0,transition:"background .15s"}}>
    <div style={{position:"absolute",top:2,left:on?15:2,width:13,height:13,borderRadius:7,background:"white",transition:"left .15s"}}/>
  </div>
  <div>
    <div style={{fontSize:11,fontWeight:500,color:"var(--color-text-primary)"}}>{label}</div>
    {sub&&<div style={{fontSize:9,color:"var(--color-text-secondary)"}}>{sub}</div>}
  </div>
</div>;

const SecT=({children})=><div style={{fontSize:10,fontWeight:500,color:"var(--color-text-secondary)",letterSpacing:".07em",
  textTransform:"uppercase",margin:"12px 0 6px",paddingBottom:4,borderBottom:"0.5px solid var(--color-border-tertiary)"}}>{children}</div>;

const R2=({children})=><div style={{display:"grid",gridTemplateColumns:"1fr 1fr",gap:5}}>{children}</div>;

// ── CLI command ────────────────────────────────────────────────────────────
function CLICmd({gen,mode,stepLoad,motorParams,satOn,avrOn,avr}){
  const[copied,setCopied]=useState(false);
  const parts=[
    "python gen_vdrop.py",`--mode ${mode}`,
    gen.genId?`--gen-id "${gen.genId}"`:"",
    `--gen-kva ${gen.kva} --gen-v ${gen.voltage}`,
    `--xd ${gen.Xd} --xdp ${gen.Xdp} --xdpp ${gen.Xdpp}`,
    `--xq ${gen.Xq} --xqp ${gen.Xqp} --xqpp ${gen.Xqpp}`,
    gen.R1?`--r1 ${gen.R1}`:"",
    satOn&&gen.SE_max?`--saturation --se-max ${gen.SE_max} --se-75max ${gen.SE_75max}`:"",
    avrOn?`--avr --ka ${avr.KA} --ta ${avr.TA} --ke ${avr.KE} --te ${avr.TE} --vrmax ${avr.VRMAX} --vrmin ${avr.VRMIN}`:"",
    (mode==="step"||mode==="both")?`--load-kw ${stepLoad.kw} --load-pf ${stepLoad.pf}`:"",
    (mode==="motor"||mode==="both")&&motorParams.baseKva?`--base-kva ${motorParams.baseKva} --base-pf ${motorParams.basePf}`:"",
    (mode==="motor"||mode==="both")?`--motor-kva ${motorParams.motorKva} --motor-pf ${motorParams.motorPf}`:"",
    "--export report.xlsx",
  ].filter(Boolean).join(" \\\n  ");
  return(<div style={{marginTop:12}}>
    <div style={{fontSize:10,fontWeight:500,color:"var(--color-text-secondary)",marginBottom:4,letterSpacing:".06em",textTransform:"uppercase"}}>Generated CLI command</div>
    <div style={{position:"relative"}}>
      <pre style={{background:"#0f172a",color:"#7dd3fc",padding:"10px 12px",borderRadius:"var(--border-radius-md)",
        fontSize:10,overflowX:"auto",lineHeight:1.7,margin:0,fontFamily:"'Fira Code',monospace",border:"1px solid #1e3a5f"}}>{parts}</pre>
      <button onClick={()=>{navigator.clipboard.writeText(parts);setCopied(true);setTimeout(()=>setCopied(false),2000);}}
        style={{position:"absolute",top:6,right:6,padding:"3px 8px",background:copied?"#059669":"#1e40af",color:"white",
          border:"none",borderRadius:4,fontSize:9,cursor:"pointer",fontWeight:700}}>{copied?"COPIED ✓":"COPY"}</button>
    </div>
  </div>);
}

// ── Main ───────────────────────────────────────────────────────────────────
export default function App(){
  const[mode,setMode]=useState("motor");
  const[gen,setGen]=useState({genId:"KATO 4P11-3600 | S/N 24793",kva:8125,voltage:4160,ratedPf:0.80,
    Xd:1.913,Xdp:0.222,Xdpp:0.169,Xq:0.871,Xqp:0.871,Xqpp:0.222,Ra:0.0069,SE_max:0,SE_75max:0});
  const[stepLoad,setStepLoad]=useState({kw:6500,pf:0.80});
  const[motorParams,setMotorParams]=useState({baseKva:720,basePf:0.85,motorKva:7164,motorPf:0.15});
  const[satOn,setSatOn]=useState(false);
  const[avrOn,setAvrOn]=useState(false);
  const[avr,setAvr]=useState({KA:200,TA:0.02,KE:1.0,TE:0.177,VRMAX:5.0,VRMIN:-1.0});
  const[minV,setMinV]=useState(0.80);
  const[results,setResults]=useState(null);
  const G=k=>v=>setGen(g=>({...g,[k]:v}));
  const S=k=>v=>setStepLoad(s=>({...s,[k]:v}));
  const M=k=>v=>setMotorParams(m=>({...m,[k]:v}));
  const A=k=>v=>setAvr(a=>({...a,[k]:v}));
  const satFit=satOn&&gen.SE_max>0&&gen.SE_75max>0?satCoeffs(gen.SE_max,gen.SE_75max):{A:0,B:0};

  const calculate=useCallback(()=>{
    const studies=mode==="both"?["step","motor"]:[mode];
    const res={};
    studies.forEach(type=>{
      const{results:periods,Spu,phi,Ib,phiB}=runStudy(gen,type,stepLoad,motorParams,satOn,satFit.A,satFit.B);
      const avrCurve=avrOn?simulateAVR(gen,avr,Spu,phi,Ib,phiB):null;
      res[type]={periods,Spu,phi,avrCurve};
    });
    setResults(res);
  },[gen,mode,stepLoad,motorParams,satOn,satFit,avrOn,avr,minV]);

  const ss={width:270,flexShrink:0,borderRight:"0.5px solid var(--color-border-tertiary)",padding:12,overflowY:"auto",background:"var(--color-background-secondary)"};
  const ms={flex:1,padding:14,overflowY:"auto",background:"var(--color-background-primary)"};
  const bgMap=["#f0fdf8","#fffcf5","var(--color-background-secondary)"];
  const bdMap=["#9fe1cb","#fac775","var(--color-border-tertiary)"];
  const clMap=["#0f6e56","#854f0b","var(--color-text-secondary)"];

  return(<div style={{display:"flex",minHeight:580,border:"0.5px solid var(--color-border-tertiary)",borderRadius:"var(--border-radius-lg)",overflow:"hidden"}}>
    <div style={ss}>
      <div style={{display:"flex",gap:3,marginBottom:10}}>
        {[["step","Step"],["motor","Motor"],["both","Both"]].map(([v,l])=><button key={v} onClick={()=>setMode(v)} style={{
          flex:1,padding:"5px 0",fontSize:11,fontWeight:500,cursor:"pointer",border:"0.5px solid",
          borderRadius:"var(--border-radius-md)",background:"var(--color-background-primary)",
          borderColor:mode===v?"var(--color-border-primary)":"var(--color-border-secondary)",
          color:mode===v?"var(--color-text-primary)":"var(--color-text-secondary)"}}>{l}</button>)}
      </div>

      <SecT>Generator</SecT>
      <Field label="ID / description"><input type="text" value={gen.genId} onChange={e=>setGen(g=>({...g,genId:e.target.value}))}
        style={{width:"100%",height:28,padding:"0 7px",fontSize:11,fontFamily:"var(--font-sans)",color:"var(--color-text-primary)",
          background:"var(--color-background-primary)",border:"0.5px solid var(--color-border-secondary)",borderRadius:"var(--border-radius-md)"}}/></Field>
      <R2>
        <Field label="Rated kVA" hint="kVA"><NumIn value={gen.kva} onChange={G("kva")}/></Field>
        <Field label="Voltage L-L" hint="V"><NumIn value={gen.voltage} onChange={G("voltage")}/></Field>
      </R2>

      <SecT>Reactances (pu, saturated)</SecT>
      <R2>
        <Field label="Xd" hint="synchronous"><NumIn value={gen.Xd} onChange={G("Xd")} step="0.001"/></Field>
        <Field label="Xq"><NumIn value={gen.Xq} onChange={G("Xq")} step="0.001"/></Field>
        <Field label="Xd'" hint="transient"><NumIn value={gen.Xdp} onChange={G("Xdp")} step="0.001"/></Field>
        <Field label="Xq'"><NumIn value={gen.Xqp} onChange={G("Xqp")} step="0.001"/></Field>
        <Field label='Xd"' hint="subtransient"><NumIn value={gen.Xdpp} onChange={G("Xdpp")} step="0.001"/></Field>
        <Field label='Xq"'><NumIn value={gen.Xqpp} onChange={G("Xqpp")} step="0.001"/></Field>
      </R2>
      <Field label="R1 — stator resistance" hint="pu"><NumIn value={gen.Ra} onChange={G("Ra")} step="0.0001"/></Field>

      <SecT>Saturation correction</SecT>
      <Tog on={satOn} onToggle={()=>setSatOn(!satOn)} label="Saturation model" sub="SE_max + SE_75max from datasheet OCC"/>
      {satOn&&<div style={{padding:8,border:"0.5px solid var(--color-border-tertiary)",borderRadius:"var(--border-radius-md)",background:"var(--color-background-primary)",marginBottom:6}}>
        <R2>
          <Field label="SE_max (1.0 pu)" hint="e.g. 0.255"><NumIn value={gen.SE_max} onChange={G("SE_max")} step="0.001"/></Field>
          <Field label="SE_75max (0.75 pu)" hint="e.g. 0.071"><NumIn value={gen.SE_75max} onChange={G("SE_75max")} step="0.001"/></Field>
        </R2>
        {satFit.A>0&&<div style={{fontSize:9,color:"var(--color-text-secondary)",fontFamily:"var(--font-mono)",lineHeight:1.5}}>
          SE(E) = {satFit.A.toFixed(5)} · exp({satFit.B.toFixed(3)} · E)</div>}
      </div>}

      <SecT>AVR response (Allen-Bradley CGCM)</SecT>
      <Tog on={avrOn} onToggle={()=>setAvrOn(!avrOn)} label="IEEE Type I AVR model" sub="Voltage recovery curve"/>
      {avrOn&&<div style={{padding:8,border:"0.5px solid var(--color-border-tertiary)",borderRadius:"var(--border-radius-md)",background:"var(--color-background-primary)",marginBottom:6}}>
        <div style={{fontSize:9,fontWeight:500,color:"var(--color-text-secondary)",textTransform:"uppercase",letterSpacing:".06em",marginBottom:5}}>From CGCM configuration</div>
        <R2>
          <Field label="KA — gain" hint="typ 200–400"><NumIn value={avr.KA} onChange={A("KA")} step="10"/></Field>
          <Field label="TA — time const (s)" hint="typ 0.01–0.05"><NumIn value={avr.TA} onChange={A("TA")} step="0.001"/></Field>
          <Field label="VRMAX (pu)" hint="typ 4–7"><NumIn value={avr.VRMAX} onChange={A("VRMAX")} step="0.1"/></Field>
          <Field label="VRMIN (pu)"><NumIn value={avr.VRMIN} onChange={A("VRMIN")} step="0.1"/></Field>
        </R2>
        <div style={{fontSize:9,fontWeight:500,color:"var(--color-text-secondary)",textTransform:"uppercase",letterSpacing:".06em",marginBottom:5,marginTop:4}}>From generator datasheet</div>
        <R2>
          <Field label="KE — exciter const"><NumIn value={avr.KE} onChange={A("KE")} step="0.01"/></Field>
          <Field label="TE — time const (s)"><NumIn value={avr.TE} onChange={A("TE")} step="0.001"/></Field>
        </R2>
      </div>}

      {(mode==="step"||mode==="both")&&<><SecT>Step load</SecT>
        <R2>
          <Field label="Active power" hint="kW"><NumIn value={stepLoad.kw} onChange={S("kw")}/></Field>
          <Field label="Power factor"><NumIn value={stepLoad.pf} onChange={S("pf")} step="0.01"/></Field>
        </R2></>}

      {(mode==="motor"||mode==="both")&&<>
        <SecT>Base load (pre-event)</SecT>
        <R2>
          <Field label="Base kVA" hint="0 = none"><NumIn value={motorParams.baseKva} onChange={M("baseKva")}/></Field>
          <Field label="Base PF"><NumIn value={motorParams.basePf} onChange={M("basePf")} step="0.01"/></Field>
        </R2>
        <SecT>Motor inrush</SecT>
        <R2>
          <Field label="Starting kVA"><NumIn value={motorParams.motorKva} onChange={M("motorKva")}/></Field>
          <Field label="Inrush PF" hint="typ 0.10–0.20"><NumIn value={motorParams.motorPf} onChange={M("motorPf")} step="0.01"/></Field>
        </R2></>}

      <Field label="Min acceptable voltage" hint="pu"><NumIn value={minV} onChange={setMinV} step="0.01"/></Field>
      <button onClick={calculate} style={{width:"100%",padding:9,marginTop:8,fontSize:13,fontWeight:500,cursor:"pointer",
        background:"var(--color-text-primary)",color:"var(--color-background-primary)",border:"none",borderRadius:"var(--border-radius-md)"}}>Calculate</button>
    </div>

    <div style={ms}>
      {!results?(<div style={{display:"flex",flexDirection:"column",alignItems:"center",justifyContent:"center",height:"100%",gap:8,color:"var(--color-text-tertiary)",fontSize:13,textAlign:"center"}}>
        <svg width="28" height="28" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="1.2" strokeLinecap="round"><polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/></svg>
        <span>Pre-loaded with KATO 4P11-3600 case study data</span>
        <span style={{fontSize:11}}>Click Calculate to run</span>
      </div>):(
        <>
          <div style={{fontSize:12,fontWeight:500,marginBottom:4}}>{gen.genId||"Generator"}</div>
          <div style={{fontSize:10,color:"var(--color-text-secondary)",fontFamily:"var(--font-mono)",marginBottom:12}}>
            {gen.kva} kVA | {gen.voltage} V | Xd"={gen.Xdpp} Xq"={gen.Xqpp} Xd'={gen.Xdp} Xq'={gen.Xqp} R1={gen.Ra} pu</div>

          {Object.entries(results).map(([type,data],idx)=>{
            const chipBg=type==="step"?"#e1f5ee":"#faeeda";
            const chipClr=type==="step"?"#0f6e56":"#854f0b";
            const olPP=data.periods[0]?.Vt, olP=data.periods[1]?.Vt;
            return(<div key={type} style={{marginBottom:16}}>
              {idx>0&&<hr style={{border:"none",borderTop:"0.5px solid var(--color-border-tertiary)",margin:"12px 0"}}/>}
              <div style={{display:"flex",alignItems:"center",marginBottom:8}}>
                <span style={{display:"inline-block",fontSize:9,fontWeight:500,padding:"2px 7px",borderRadius:"var(--border-radius-md)",background:chipBg,color:chipClr,marginRight:8}}>{type==="step"?"Step load":"Motor starting"}</span>
                <span style={{fontSize:10,color:"var(--color-text-secondary)"}}>
                  {type==="step"?`${(stepLoad.kw/stepLoad.pf).toFixed(0)} kVA at PF ${stepLoad.pf}`
                    :`${motorParams.baseKva>0?motorParams.baseKva+" kVA base + ":""}${motorParams.motorKva} kVA inrush`}
                </span>
              </div>

              {data.periods.map((p,i)=>{
                const dip=p.Vt?(1-p.Vt)*100:null, pass=p.Vt&&p.Vt>=minV;
                const simpDipV=p.Vsimp?(1-p.Vsimp)*100:null;
                return(<div key={p.label} style={{border:`0.5px solid ${bdMap[i]}`,borderRadius:"var(--border-radius-md)",background:bgMap[i],padding:"9px 11px",marginBottom:6,display:"flex",justifyContent:"space-between",alignItems:"center"}}>
                  <div>
                    <div style={{fontSize:11,fontWeight:500}}>{p.label} <span style={{fontSize:9,color:"var(--color-text-secondary)"}}>({p.sub})</span></div>
                    <div style={{fontSize:9,color:"var(--color-text-secondary)",fontFamily:"var(--font-mono)",marginTop:1}}>
                      Xd={p.Xd?.toFixed(3)} Xq={p.Xq?.toFixed(3)} E₀={p.Eq?.toFixed(4)} pu
                      {simpDipV!=null&&dip!=null?`  |  simplified: ${simpDipV.toFixed(1)}% (Δ${(dip-simpDipV).toFixed(1)}%)`:""}
                    </div>
                  </div>
                  <div style={{textAlign:"right"}}>
                    {p.Vt==null?<div style={{fontSize:12,fontWeight:500,color:"var(--color-text-danger)"}}>UNSTABLE</div>
                      :<><div style={{fontSize:20,fontWeight:500,color:clMap[i]}}>{dip.toFixed(2)}%</div>
                        <div style={{fontSize:10,color:"var(--color-text-secondary)"}}>{p.Vt.toFixed(4)} pu | {(p.Vt*gen.voltage).toFixed(0)} V</div>
                        <div style={{fontSize:10,fontWeight:500,color:pass?"var(--color-text-success)":"var(--color-text-danger)",marginTop:2}}>{pass?"✓ pass":"✗ fail"}</div></>}
                  </div>
                </div>);
              })}

              {avrOn&&data.avrCurve&&(<div style={{border:"0.5px solid var(--color-border-tertiary)",borderRadius:"var(--border-radius-md)",padding:"10px 12px",marginTop:8}}>
                <div style={{display:"flex",justifyContent:"space-between",alignItems:"center",marginBottom:8}}>
                  <span style={{fontSize:11,fontWeight:500}}>AVR response — CGCM (KA={avr.KA}, TA={avr.TA}s, KE={avr.KE}, TE={avr.TE}s)</span>
                  <div style={{display:"flex",gap:10,fontSize:9,color:"var(--color-text-secondary)"}}>
                    <span style={{display:"flex",alignItems:"center",gap:3}}>
                      <svg width="16" height="5"><line x1="0" y1="2.5" x2="16" y2="2.5" stroke="#185fa5" strokeWidth="2"/></svg>AVR on</span>
                    <span style={{display:"flex",alignItems:"center",gap:3}}>
                      <svg width="16" height="5"><line x1="0" y1="2.5" x2="16" y2="2.5" stroke="#5dcaa5" strokeWidth="1" strokeDasharray="4,2"/></svg>Open-loop</span>
                  </div>
                </div>
                <VoltageChart times={data.avrCurve.times} volts={data.avrCurve.volts} olPP={olPP} olP={olP} minV={minV} genVolt={gen.voltage}/>
                <div style={{fontSize:10,color:"var(--color-text-secondary)",marginTop:8,lineHeight:1.5}}>
                  Peak dip (0–50 ms) is unchanged — the CGCM AVR cannot respond in the subtransient window.
                  AVR begins boosting excitation from ~50 ms, recovering voltage ahead of the open-loop Xd' floor.
                  Use peak dip for relay pickup and UPS settings. Use recovery curve for motor starting acceptance.
                </div>
              </div>)}
            </div>);
          })}

          <div style={{fontSize:10,color:"var(--color-text-secondary)",background:"var(--color-background-secondary)",borderRadius:"var(--border-radius-md)",padding:"7px 9px",lineHeight:1.5}}>
            Salient-pole NR closes ~65% of gap vs manufacturer. Remaining ~1–2% is dynamic saturation. Apply 10–15% margin for final decisions.
          </div>
          <CLICmd gen={gen} mode={mode} stepLoad={stepLoad} motorParams={motorParams} satOn={satOn} avrOn={avrOn} avr={avr}/>
        </>
      )}
    </div>
  </div>);
}
