import { useState, useCallback } from "react";

// ── Salient-pole Newton-Raphson solver (JS port of gen_vdrop.py) ──────────
function calcEqPre(Vt, Ibase, phiBase, Xd, Xq, Ra) {
  if (Ibase < 1e-9) return { Eq: Vt, delta: 0 };
  let delta = Math.atan2(Xq * Ibase * Math.cos(phiBase), Vt + Xq * Ibase * Math.sin(phiBase));
  for (let i = 0; i < 100; i++) {
    const Id = Ibase * Math.sin(delta + phiBase);
    const Iq = Ibase * Math.cos(delta + phiBase);
    const f2 = Vt * Math.sin(delta) - Xq * Iq + Ra * Id;
    const df2 = Vt * Math.cos(delta) - Xq * (-Ibase * Math.sin(delta + phiBase)) + Ra * (Ibase * Math.cos(delta + phiBase));
    if (Math.abs(df2) < 1e-15) break;
    const step = f2 / df2; delta -= step;
    if (Math.abs(step) < 1e-12) break;
  }
  const Id = Ibase * Math.sin(delta + phiBase);
  const Iq = Ibase * Math.cos(delta + phiBase);
  const Eq = Vt * Math.cos(delta) + Ra * Iq + Xd * Id;
  return { Eq, delta };
}

function satCoeffs(SE_max, SE_75) {
  if (SE_75 <= 0 || SE_max <= SE_75) return { A: 0, B: 0 };
  const B = Math.log(SE_max / SE_75) / 0.25;
  const A = SE_75 / Math.exp(0.75 * B);
  return { A, B };
}

function calcEqPreSat(Vt, Ibase, phiBase, Xd, Xq, Ra, A, B) {
  const { Eq: EqLin, delta } = calcEqPre(Vt, Ibase, phiBase, Xd, Xq, Ra);
  if (A <= 0 || Ibase < 1e-9) return { Eq: EqLin, delta };
  const IdPre = Ibase * Math.sin(delta + phiBase);
  const seCorr = A * Math.exp(B * EqLin) * Xd * IdPre;
  return { Eq: EqLin + seCorr, delta };
}

function newtonRaphson(Eq, Spu, phi, Xd, Xq, Ra) {
  let V = 0.90, d = 15 * Math.PI / 180;
  for (let i = 0; i < 200; i++) {
    const I = Spu / V, Id = I * Math.sin(d + phi), Iq = I * Math.cos(d + phi);
    const f1 = Eq - V * Math.cos(d) - Ra * Iq - Xd * Id;
    const f2 = V * Math.sin(d) - Xq * Iq + Ra * Id;
    if (Math.abs(f1) < 1e-9 && Math.abs(f2) < 1e-9) return { Vt: V, delta: d, ok: true };
    const dIdV = -I / V;
    const J00 = -Math.cos(d) - Ra * dIdV * Math.cos(d + phi) - Xd * dIdV * Math.sin(d + phi);
    const J01 = V * Math.sin(d) - Ra * (-I * Math.sin(d + phi)) - Xd * (I * Math.cos(d + phi));
    const J10 = Math.sin(d) - Xq * dIdV * Math.cos(d + phi) + Ra * dIdV * Math.sin(d + phi);
    const J11 = V * Math.cos(d) - Xq * (-I * Math.sin(d + phi)) + Ra * (I * Math.cos(d + phi));
    const det = J00 * J11 - J01 * J10;
    if (Math.abs(det) < 1e-15) return { Vt: null, ok: false };
    const dV = -(J11 * f1 - J01 * f2) / det;
    const dd = -(-J10 * f1 + J00 * f2) / det;
    V = Math.max(0.005, V + dV); d += dd;
  }
  return { Vt: null, ok: false };
}

function simplifiedDip(X, Ipu, pf) {
  const sinPhi = Math.sqrt(Math.max(0, 1 - pf * pf));
  const disc = 1 - (X * Ipu * pf) ** 2;
  if (disc < 0) return null;
  return Math.sqrt(disc) - X * Ipu * sinPhi;
}

function runStudy(gen, mode, stepLoad, motorParams) {
  const results = [];
  const periods = mode === "step"
    ? [["Subtransient  (0 – 50 ms)", gen.Xdpp, gen.Xqpp],
       ["Transient  (50 – 500 ms)", gen.Xdp, gen.Xqp],
       ["Steady-State  (> 1 s)", gen.Xd, gen.Xq]]
    : [["Subtransient  (0 – 50 ms)", gen.Xdpp, gen.Xqpp],
       ["Transient  (50 – 500 ms)", gen.Xdp, gen.Xqp],
       ["Steady-State  (> 1 s)", gen.Xd, gen.Xq]];

  let Spu, phi, Ibase = 0, phiBase = 0;
  if (mode === "step") {
    const kva = stepLoad.kw / stepLoad.pf;
    Spu = kva / gen.kva;
    phi = Math.acos(stepLoad.pf);
  } else {
    const P = (motorParams.baseKva || 0) * (motorParams.basePf || 0.85) + motorParams.motorKva * motorParams.motorPf;
    const Q = (motorParams.baseKva || 0) * Math.sqrt(Math.max(0, 1 - (motorParams.basePf || 0.85) ** 2))
            + motorParams.motorKva * Math.sqrt(Math.max(0, 1 - motorParams.motorPf ** 2));
    Spu = Math.sqrt(P ** 2 + Q ** 2) / gen.kva;
    phi = Math.atan2(Q, P);
    Ibase = (motorParams.baseKva || 0) / gen.kva;
    phiBase = Math.acos(motorParams.basePf || 0.85);
  }

  let satA = 0, satB = 0;
  if (gen.useSat && gen.SE_max > 0 && gen.SE_75max > 0) {
    ({ A: satA, B: satB } = satCoeffs(gen.SE_max, gen.SE_75max));
  }

  for (const [label, Xd, Xq] of periods) {
    if (!Xd || !Xq) { results.push({ label, Xd, Xq, Eq: null, Vt: null }); continue; }
    let EqResult;
    if (mode === "motor" && gen.useSat && satA > 0) {
      EqResult = calcEqPreSat(1.0, Ibase, phiBase, Xd, Xq, gen.R1, satA, satB);
    } else {
      EqResult = calcEqPre(1.0, Ibase, phiBase, Xd, Xq, gen.R1);
    }
    const { Eq } = EqResult;
    const { Vt, ok } = newtonRaphson(Eq, Spu, phi, Xd, Xq, gen.R1);
    const simpPf = mode === "step" ? stepLoad.pf : (Spu > 0 ? Math.cos(phi) : 0.8);
    const Vsimp = simplifiedDip(Xd, Spu, simpPf);
    results.push({ label, Xd, Xq, Eq, Vt: ok ? Vt : null, Vsimp });
  }
  return { results, Spu, phi };
}

// ── UI Components ─────────────────────────────────────────────────────────
const TEAL = "#0d9488"; const AMBER = "#d97706"; const ROSE = "#e11d48";
const SLATE = "#0f172a"; const BLUE = "#1d4ed8";

const sectionColors = [
  { bg: "#d1fae5", border: "#059669", label: "#065f46" },
  { bg: "#fef3c7", border: "#d97706", label: "#92400e" },
  { bg: "#fee2e2", border: "#ef4444", label: "#991b1b" },
];

function Input({ label, value, onChange, unit, hint, type = "number", step = "any", small }) {
  return (
    <div style={{ marginBottom: 10 }}>
      <div style={{ fontSize: 11, fontWeight: 600, color: "#64748b", marginBottom: 3, letterSpacing: "0.05em", textTransform: "uppercase" }}>{label}</div>
      <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
        <input
          type={type} value={value} step={step}
          onChange={e => onChange(type === "number" ? parseFloat(e.target.value) || 0 : e.target.value)}
          style={{
            width: small ? 80 : 120, padding: "6px 10px", border: "1.5px solid #cbd5e1",
            borderRadius: 6, fontSize: 13, fontFamily: "monospace", color: BLUE,
            fontWeight: 700, background: "#f0f9ff", outline: "none",
          }}
        />
        {unit && <span style={{ fontSize: 12, color: "#94a3b8", fontStyle: "italic" }}>{unit}</span>}
      </div>
      {hint && <div style={{ fontSize: 10, color: "#94a3b8", marginTop: 2 }}>{hint}</div>}
    </div>
  );
}

function Toggle({ label, checked, onChange, hint }) {
  return (
    <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
      <div
        onClick={() => onChange(!checked)}
        style={{
          width: 40, height: 22, borderRadius: 11, cursor: "pointer",
          background: checked ? TEAL : "#cbd5e1", transition: "background 0.2s",
          position: "relative", flexShrink: 0,
        }}
      >
        <div style={{
          position: "absolute", top: 3, left: checked ? 21 : 3,
          width: 16, height: 16, borderRadius: 8, background: "white",
          transition: "left 0.2s", boxShadow: "0 1px 3px rgba(0,0,0,0.2)"
        }} />
      </div>
      <div>
        <div style={{ fontSize: 12, fontWeight: 600, color: SLATE }}>{label}</div>
        {hint && <div style={{ fontSize: 10, color: "#94a3b8" }}>{hint}</div>}
      </div>
    </div>
  );
}

function Section({ title, number, children, accent = "#1d4ed8" }) {
  return (
    <div style={{ marginBottom: 20 }}>
      <div style={{
        display: "flex", alignItems: "center", gap: 10, marginBottom: 12,
        borderBottom: `2px solid ${accent}`, paddingBottom: 6,
      }}>
        <div style={{
          width: 24, height: 24, borderRadius: 12, background: accent,
          display: "flex", alignItems: "center", justifyContent: "center",
          fontSize: 11, fontWeight: 800, color: "white", flexShrink: 0,
        }}>{number}</div>
        <div style={{ fontSize: 13, fontWeight: 700, color: SLATE, letterSpacing: "0.03em" }}>{title}</div>
      </div>
      <div style={{ paddingLeft: 8 }}>{children}</div>
    </div>
  );
}

function ResultRow({ period, Xd, Xq, Eq, Vt, Vsimp, genV, idx }) {
  const c = sectionColors[idx % 3];
  const dip = Vt ? (1 - Vt) * 100 : null;
  const vll = Vt ? Vt * genV : null;
  const simpDip = Vsimp ? (1 - Vsimp) * 100 : null;
  const unstable = Vt === null;
  return (
    <div style={{
      background: c.bg, border: `1.5px solid ${c.border}`, borderRadius: 8,
      padding: "10px 14px", marginBottom: 8,
    }}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "flex-start", flexWrap: "wrap", gap: 8 }}>
        <div>
          <div style={{ fontSize: 12, fontWeight: 700, color: c.label }}>{period}</div>
          <div style={{ fontSize: 10, color: "#64748b", marginTop: 2 }}>
            Xd = {Xd?.toFixed(3)} pu  |  Xq = {Xq?.toFixed(3)} pu  |  E₀ = {Eq?.toFixed(5)} pu
          </div>
        </div>
        <div style={{ textAlign: "right" }}>
          {unstable ? (
            <div style={{ fontSize: 13, fontWeight: 800, color: "#991b1b" }}>UNSTABLE</div>
          ) : (
            <>
              <div style={{ fontSize: 20, fontWeight: 800, color: c.label }}>{dip?.toFixed(2)}% dip</div>
              <div style={{ fontSize: 11, color: "#64748b" }}>
                V_t = {Vt?.toFixed(4)} pu  |  {vll?.toFixed(0)} V L-L
              </div>
            </>
          )}
          {simpDip != null && (
            <div style={{ fontSize: 10, color: "#94a3b8", marginTop: 2 }}>
              Simplified: {simpDip.toFixed(2)}% {!unstable && `(Δ = ${(dip - simpDip).toFixed(2)}%)`}
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

function CLICommand({ gen, mode, stepLoad, motorParams }) {
  const [copied, setCopied] = useState(false);
  const parts = [
    "python gen_vdrop.py",
    `--mode ${mode}`,
    gen.genId ? `--gen-id "${gen.genId}"` : "",
    `--gen-kva ${gen.kva}`,
    `--gen-v ${gen.voltage}`,
    gen.Xd ? `--xd ${gen.Xd}` : "",
    gen.Xdp ? `--xdp ${gen.Xdp}` : "",
    gen.Xdpp ? `--xdpp ${gen.Xdpp}` : "",
    gen.Xq ? `--xq ${gen.Xq}` : "",
    gen.Xqp ? `--xqp ${gen.Xqp}` : "",
    gen.Xqpp ? `--xqpp ${gen.Xqpp}` : "",
    gen.R1 ? `--r1 ${gen.R1}` : "",
    gen.useSat && gen.SE_max ? `--saturation --se-max ${gen.SE_max} --se-75max ${gen.SE_75max}` : "",
    mode === "step" ? `--load-kw ${stepLoad.kw} --load-pf ${stepLoad.pf}` : "",
    mode === "motor" && motorParams.baseKva ? `--base-kva ${motorParams.baseKva} --base-pf ${motorParams.basePf}` : "",
    mode === "motor" ? `--motor-kva ${motorParams.motorKva} --motor-pf ${motorParams.motorPf}` : "",
    `--export report.xlsx`,
  ].filter(Boolean).join(" \\\n  ");

  const copy = () => { navigator.clipboard.writeText(parts); setCopied(true); setTimeout(() => setCopied(false), 2000); };
  return (
    <div style={{ marginTop: 16 }}>
      <div style={{ fontSize: 11, fontWeight: 700, color: "#64748b", marginBottom: 6, letterSpacing: "0.05em", textTransform: "uppercase" }}>
        Generated CLI Command
      </div>
      <div style={{ position: "relative" }}>
        <pre style={{
          background: "#0f172a", color: "#7dd3fc", padding: "12px 14px", borderRadius: 8,
          fontSize: 11, overflowX: "auto", lineHeight: 1.7, margin: 0,
          fontFamily: "'Fira Code', 'Cascadia Code', monospace",
          border: "1px solid #1e3a5f",
        }}>{parts}</pre>
        <button onClick={copy} style={{
          position: "absolute", top: 8, right: 8, padding: "4px 10px",
          background: copied ? "#059669" : "#1e40af", color: "white",
          border: "none", borderRadius: 4, fontSize: 10, cursor: "pointer",
          fontWeight: 700, letterSpacing: "0.05em",
        }}>{copied ? "COPIED ✓" : "COPY"}</button>
      </div>
    </div>
  );
}

// ── Main App ───────────────────────────────────────────────────────────────
export default function App() {
  const [mode, setMode] = useState("both");
  const [gen, setGen] = useState({
    genId: "", kva: 8125, voltage: 4160, ratedPf: 0.80, freq: 60,
    Xd: 1.913, Xdp: 0.222, Xdpp: 0.169,
    Xq: 0.871, Xqp: 0.871, Xqpp: 0.222,
    R1: 0.0069, X2: 0.195, X0: 0.085,
    SE_max: 0, SE_75max: 0, useSat: false,
  });
  const [stepLoad, setStepLoad] = useState({ kw: 6500, pf: 0.80 });
  const [motorParams, setMotorParams] = useState({
    baseKva: 720, basePf: 0.85,
    motorKva: 7164, motorPf: 0.15,
  });
  const [results, setResults] = useState(null);
  const [minV, setMinV] = useState(0.80);

  const G = (k) => (v) => setGen(g => ({ ...g, [k]: v }));
  const S = (k) => (v) => setStepLoad(s => ({ ...s, [k]: v }));
  const M = (k) => (v) => setMotorParams(m => ({ ...m, [k]: v }));

  const calculate = useCallback(() => {
    const res = {};
    if (mode === "step" || mode === "both") {
      res.step = runStudy(gen, "step", stepLoad, motorParams);
    }
    if (mode === "motor" || mode === "both") {
      res.motor = runStudy(gen, "motor", stepLoad, motorParams);
    }
    setResults(res);
  }, [gen, mode, stepLoad, motorParams]);

  const satInfo = gen.useSat && gen.SE_max > 0 && gen.SE_75max > 0
    ? satCoeffs(gen.SE_max, gen.SE_75max) : null;

  return (
    <div style={{ fontFamily: "'Inter', system-ui, sans-serif", background: "#f8fafc", minHeight: "100vh" }}>
      {/* Header */}
      <div style={{ background: SLATE, padding: "18px 24px", borderBottom: "3px solid #1d4ed8" }}>
        <div style={{ fontSize: 18, fontWeight: 800, color: "white", letterSpacing: "-0.02em" }}>
          Generator Voltage Drop Calculator
        </div>
        <div style={{ fontSize: 11, color: "#7dd3fc", marginTop: 3, letterSpacing: "0.05em" }}>
          SALIENT-POLE NEWTON-RAPHSON  •  GEN_VDROP.PY  •  GITHUB: NAEROLLS/GEN-VDROP-CALCULATOR
        </div>
      </div>

      <div style={{ display: "flex", gap: 0, minHeight: "calc(100vh - 70px)" }}>
        {/* ── Input Panel ── */}
        <div style={{
          width: 320, flexShrink: 0, background: "white",
          borderRight: "1px solid #e2e8f0", overflowY: "auto",
          padding: 20,
        }}>
          {/* Study mode */}
          <div style={{ marginBottom: 20 }}>
            <div style={{ fontSize: 11, fontWeight: 700, color: "#64748b", marginBottom: 8, letterSpacing: "0.05em", textTransform: "uppercase" }}>Study Mode</div>
            <div style={{ display: "flex", gap: 6 }}>
              {[["step", "Step Load"], ["motor", "Motor Start"], ["both", "Both"]].map(([v, l]) => (
                <button key={v} onClick={() => setMode(v)} style={{
                  flex: 1, padding: "7px 4px", borderRadius: 6, fontSize: 11, fontWeight: 700,
                  cursor: "pointer", border: "1.5px solid",
                  background: mode === v ? BLUE : "white",
                  color: mode === v ? "white" : "#64748b",
                  borderColor: mode === v ? BLUE : "#cbd5e1",
                }}>{l}</button>
              ))}
            </div>
          </div>

          {/* Generator nameplate */}
          <Section number="1" title="Generator Nameplate" accent={BLUE}>
            <Input label="Generator ID" value={gen.genId} onChange={G("genId")} type="text" hint="Model / serial / project ref" />
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0 12px" }}>
              <Input label="Rated kVA" value={gen.kva} onChange={G("kva")} unit="kVA" small />
              <Input label="Voltage (L-L)" value={gen.voltage} onChange={G("voltage")} unit="V" small />
              <Input label="Rated PF" value={gen.ratedPf} onChange={G("ratedPf")} step="0.01" small />
              <Input label="Frequency" value={gen.freq} onChange={G("freq")} unit="Hz" small />
            </div>
            <div style={{ fontSize: 10, color: "#94a3b8", marginTop: 4 }}>
              Rated current: {gen.kva && gen.voltage ? (gen.kva * 1000 / (Math.sqrt(3) * gen.voltage)).toFixed(1) : "—"} A
            </div>
          </Section>

          {/* Reactances */}
          <Section number="2" title="Reactances  (pu, saturated)" accent="#7c3aed">
            <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0 12px" }}>
              <Input label="Xd" value={gen.Xd} onChange={G("Xd")} unit="pu" small hint="Synchronous" />
              <Input label="Xq" value={gen.Xq} onChange={G("Xq")} unit="pu" small hint="Synchronous" />
              <Input label="Xd'" value={gen.Xdp} onChange={G("Xdp")} unit="pu" small hint="Transient" />
              <Input label="Xq'" value={gen.Xqp} onChange={G("Xqp")} unit="pu" small hint="Transient" />
              <Input label='Xd"' value={gen.Xdpp} onChange={G("Xdpp")} unit="pu" small hint="Subtransient" />
              <Input label='Xq"' value={gen.Xqpp} onChange={G("Xqpp")} unit="pu" small hint="Subtransient" />
            </div>
            <Input label="R1  (stator resistance)" value={gen.R1} onChange={G("R1")} unit="pu" step="0.0001" hint="Positive sequence resistance" />

            {/* Saturation toggle */}
            <div style={{ marginTop: 12, padding: "10px 12px", background: "#f0fdf4", borderRadius: 8, border: "1px solid #86efac" }}>
              <Toggle
                label="Saturation Correction"
                checked={gen.useSat}
                onChange={G("useSat")}
                hint="Uses SE_max and SE_75max from datasheet OCC"
              />
              {gen.useSat && (
                <>
                  <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0 12px" }}>
                    <Input label="SE_max  (at 1.0 pu)" value={gen.SE_max} onChange={G("SE_max")} step="0.001" small hint="e.g. 0.255" />
                    <Input label="SE_75max (at 0.75 pu)" value={gen.SE_75max} onChange={G("SE_75max")} step="0.001" small hint="e.g. 0.071" />
                  </div>
                  {satInfo && (
                    <div style={{ fontSize: 10, color: "#065f46", background: "#dcfce7", padding: "5px 8px", borderRadius: 5, marginTop: 4 }}>
                      SE(E) = {satInfo.A.toFixed(5)} · exp({satInfo.B.toFixed(3)} · E)
                      <br />SE(1.0) = {(satInfo.A * Math.exp(satInfo.B)).toFixed(4)}  |  SE(0.75) = {(satInfo.A * Math.exp(0.75 * satInfo.B)).toFixed(4)}
                      <br />Correction active only for pre-loaded motor start study
                    </div>
                  )}
                </>
              )}
            </div>
          </Section>

          {/* Step load */}
          {(mode === "step" || mode === "both") && (
            <Section number="3" title="Step Load" accent={TEAL}>
              <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0 12px" }}>
                <Input label="Active Power" value={stepLoad.kw} onChange={S("kw")} unit="kW" small />
                <Input label="Power Factor" value={stepLoad.pf} onChange={S("pf")} step="0.01" small />
              </div>
              <div style={{ fontSize: 10, color: "#94a3b8" }}>
                Load kVA: {stepLoad.pf ? (stepLoad.kw / stepLoad.pf).toFixed(1) : "—"}  |  {gen.kva ? ((stepLoad.kw / stepLoad.pf / gen.kva) * 100).toFixed(1) : "—"}% of generator
              </div>
            </Section>
          )}

          {/* Motor starting */}
          {(mode === "motor" || mode === "both") && (
            <Section number={mode === "both" ? "4" : "3"} title="Motor Starting" accent={AMBER}>
              <div style={{ fontSize: 11, fontWeight: 600, color: "#78716c", marginBottom: 6 }}>Pre-existing base load</div>
              <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0 12px" }}>
                <Input label="Base Load" value={motorParams.baseKva} onChange={M("baseKva")} unit="kVA" small hint="0 = no pre-load" />
                <Input label="Base PF" value={motorParams.basePf} onChange={M("basePf")} step="0.01" small />
              </div>
              <div style={{ fontSize: 11, fontWeight: 600, color: "#78716c", marginBottom: 6, marginTop: 8 }}>Motor inrush</div>
              <div style={{ display: "grid", gridTemplateColumns: "1fr 1fr", gap: "0 12px" }}>
                <Input label="Starting kVA" value={motorParams.motorKva} onChange={M("motorKva")} unit="kVA" small hint="See unit converter" />
                <Input label="Inrush PF" value={motorParams.motorPf} onChange={M("motorPf")} step="0.01" small hint="Typ 0.10–0.20" />
              </div>
              <div style={{ fontSize: 10, color: "#94a3b8" }}>
                {gen.kva ? `Motor start = ${((motorParams.motorKva / gen.kva) * 100).toFixed(1)}% of generator rating` : ""}
              </div>
            </Section>
          )}

          {/* Acceptability threshold */}
          <div style={{ padding: "10px 12px", background: "#fff7ed", borderRadius: 8, border: "1px solid #fdba74", marginBottom: 16 }}>
            <Input label="Min Acceptable Voltage" value={minV} onChange={setMinV} unit="pu" step="0.01"
              hint="0.80 pu = motor starting  |  0.85–0.90 pu = PLC/sensitive loads" />
          </div>

          {/* Calculate button */}
          <button onClick={calculate} style={{
            width: "100%", padding: "12px 0", background: BLUE,
            color: "white", border: "none", borderRadius: 8, fontSize: 14,
            fontWeight: 800, cursor: "pointer", letterSpacing: "0.05em",
            boxShadow: "0 4px 12px rgba(29,78,216,0.3)", transition: "opacity 0.15s",
          }}>
            ▶  CALCULATE
          </button>
        </div>

        {/* ── Results Panel ── */}
        <div style={{ flex: 1, overflowY: "auto", padding: 24 }}>
          {!results ? (
            <div style={{ display: "flex", flexDirection: "column", alignItems: "center", justifyContent: "center", height: "100%", color: "#94a3b8" }}>
              <div style={{ fontSize: 48, marginBottom: 16 }}>⚡</div>
              <div style={{ fontSize: 16, fontWeight: 600 }}>Enter generator data and click Calculate</div>
              <div style={{ fontSize: 13, marginTop: 8 }}>Pre-loaded with KATO 4P11-3600 example data</div>
            </div>
          ) : (
            <>
              {/* Step load results */}
              {results.step && (
                <div style={{ marginBottom: 28 }}>
                  <div style={{ fontSize: 15, fontWeight: 800, color: SLATE, marginBottom: 14, display: "flex", alignItems: "center", gap: 8 }}>
                    <span style={{ background: TEAL, color: "white", padding: "3px 10px", borderRadius: 4, fontSize: 11 }}>STEP LOAD</span>
                    No pre-existing load  |  {(stepLoad.kw / stepLoad.pf).toFixed(0)} kVA @ PF {stepLoad.pf}  ({((stepLoad.kw / stepLoad.pf / gen.kva) * 100).toFixed(1)}% of generator)
                  </div>
                  {results.step.results.map((r, i) => (
                    <ResultRow key={i} {...r} genV={gen.voltage} idx={i} />
                  ))}
                </div>
              )}

              {/* Motor starting results */}
              {results.motor && (
                <div style={{ marginBottom: 28 }}>
                  <div style={{ fontSize: 15, fontWeight: 800, color: SLATE, marginBottom: 6, display: "flex", alignItems: "center", gap: 8 }}>
                    <span style={{ background: AMBER, color: "white", padding: "3px 10px", borderRadius: 4, fontSize: 11 }}>MOTOR STARTING</span>
                    {motorParams.baseKva ? `${motorParams.baseKva} kVA base + ` : "No pre-load + "}{motorParams.motorKva} kVA inrush
                  </div>
                  {gen.useSat && gen.SE_max > 0 && (
                    <div style={{ fontSize: 11, color: "#065f46", background: "#dcfce7", padding: "4px 10px", borderRadius: 5, marginBottom: 10, display: "inline-block" }}>
                      ✓ Saturation correction active  (SE_max={gen.SE_max}, SE_75max={gen.SE_75max})
                    </div>
                  )}
                  {results.motor.results.map((r, i) => (
                    <ResultRow key={i} {...r} genV={gen.voltage} idx={i} />
                  ))}
                </div>
              )}

              {/* Acceptability check */}
              {(results.step || results.motor) && (
                <div style={{ background: "white", border: "1px solid #e2e8f0", borderRadius: 10, padding: 16, marginBottom: 20 }}>
                  <div style={{ fontSize: 13, fontWeight: 700, color: SLATE, marginBottom: 12 }}>
                    Acceptability Check  (min = {minV} pu)
                  </div>
                  {[results.step, results.motor].filter(Boolean).map((study, si) => (
                    study.results.slice(0, 2).map((r, i) => {
                      const c = sectionColors[i];
                      const pass = r.Vt !== null && r.Vt >= minV;
                      const unstable = r.Vt === null;
                      return (
                        <div key={`${si}-${i}`} style={{
                          display: "flex", justifyContent: "space-between", alignItems: "center",
                          padding: "7px 10px", borderRadius: 6, marginBottom: 6,
                          background: unstable ? "#fef2f2" : pass ? "#f0fdf4" : "#fef2f2",
                          border: `1px solid ${unstable ? "#fca5a5" : pass ? "#86efac" : "#fca5a5"}`,
                        }}>
                          <div style={{ fontSize: 12, color: "#475569" }}>
                            {si === 0 ? "Step Load" : "Motor Start"}  —  {r.label}
                          </div>
                          <div style={{
                            fontSize: 13, fontWeight: 800,
                            color: unstable ? "#991b1b" : pass ? "#166534" : "#991b1b",
                          }}>
                            {unstable ? "UNSTABLE" : pass ? `✔ PASS  (${((1 - r.Vt) * 100).toFixed(2)}% dip)` : `✖ FAIL  (${((1 - r.Vt) * 100).toFixed(2)}% dip)`}
                          </div>
                        </div>
                      );
                    })
                  ))}
                  <div style={{ fontSize: 10, color: "#94a3b8", marginTop: 8 }}>
                    Apply 10–15% margin: ~{(dip => `limit ${(minV + 0.85*(1-minV)).toFixed(2)} pu practical threshold`)()} — NR formula underestimates dip vs manufacturer ~1–2%
                  </div>
                </div>
              )}

              {/* Accuracy note */}
              <div style={{ background: "#eff6ff", border: "1px solid #bfdbfe", borderRadius: 8, padding: "10px 14px", marginBottom: 20, fontSize: 11, color: "#1e40af", lineHeight: 1.6 }}>
                <strong>Accuracy:</strong> Salient-pole NR closes ~65% of gap vs manufacturer published values. Remaining ~1–2% is dynamic saturation (not modeled with 2-point SE data). Apply 10–15% margin on acceptable dip threshold for final protection engineering decisions.
              </div>

              {/* CLI command */}
              <CLICommand gen={gen} mode={mode} stepLoad={stepLoad} motorParams={motorParams} />
            </>
          )}
        </div>
      </div>
    </div>
  );
}
