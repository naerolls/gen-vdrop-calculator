# gen-vdrop-calculator

Generator voltage drop calculator using a full **salient-pole Newton-Raphson** solution.

## What it does

- **Step load study** — sudden balanced three-phase load applied to a generator at no-load
- **Motor starting study** — pre-existing base load + motor inrush, with pre-event internal EMF (E₀) calculated from actual base load conditions
- **Salient-pole NR solver** — includes Xd, Xq, Ra (stator resistance); significantly more accurate than the simplified cylindrical-rotor formula
- **Simplified formula** — shown alongside NR results for reference
- **Excel report export** — formatted multi-sheet workbook with summary + per-study detail sheets
- **Unit conversions** — HP→kW, %→pu, Ω→pu, FLA+inrush multiplier→starting kVA, V_LL↔V_LN

## Usage

### Interactive mode (prompted inputs)
```bash
python gen_vdrop.py
```

### Step load study — CLI
```bash
python gen_vdrop.py --mode step \
  --gen-id "My Generator" --gen-kva 8125 --gen-v 4160 \
  --xd 1.913 --xdp 0.222 --xdpp 0.169 \
  --xq 0.871 --xqp 0.871 --xqpp 0.222 --r1 0.0069 \
  --load-kw 6500 --load-pf 0.80
```

### Motor starting study — from starting kVA
```bash
python gen_vdrop.py --mode motor \
  --gen-kva 8125 --gen-v 4160 \
  --xd 1.913 --xdp 0.222 --xdpp 0.169 \
  --xq 0.871 --xqp 0.871 --xqpp 0.222 --r1 0.0069 \
  --base-kva 720 --base-pf 0.85 \
  --motor-kva 7164 --motor-pf 0.15
```

### Motor starting study — from HP + FLA (built-in unit conversion)
```bash
python gen_vdrop.py --mode motor \
  --gen-kva 8125 --gen-v 4160 \
  --xd 1.913 --xdp 0.222 --xdpp 0.169 \
  --xq 0.871 --xqp 0.871 --xqpp 0.222 --r1 0.0069 \
  --base-kva 720 --base-pf 0.85 \
  --hp 1250 --fla 156 --v-motor 4160 --inrush-mult 6.5 --motor-pf 0.15
```

### Export to Excel
Add `--export report.xlsx` to any command above.

### Run both studies + export
```bash
python gen_vdrop.py --mode both \
  --gen-kva 8125 --gen-v 4160 \
  --xd 1.913 --xdp 0.222 --xdpp 0.169 \
  --xq 0.871 --xqp 0.871 --xqpp 0.222 --r1 0.0069 \
  --load-kw 6500 --load-pf 0.80 \
  --base-kva 720 --base-pf 0.85 \
  --motor-kva 7164 --motor-pf 0.15 \
  --export report.xlsx
```

### Unit conversions
```bash
python gen_vdrop.py --convert hp 1250
python gen_vdrop.py --convert pct 16.9
python gen_vdrop.py --convert starting-kva 156 6.5 4160
python gen_vdrop.py --convert ohms 0.071 8125 4160
```

## Dependencies

- Python 3.8+
- `openpyxl` for Excel export: `pip install openpyxl`

No dependencies required for console-only output.

## Theory

### Simplified formula (cylindrical rotor, Ra = 0)
```
|V_t| = √(1 − (X·I·cosφ)²) − X·I·sinφ
```
Fast but optimistic — underestimates dip by 25–50% vs manufacturer models.

### Salient-pole Newton-Raphson
Solves simultaneously:
```
f1: E_q − V_t·cos(δ) − Ra·Iq − Xd·Id = 0
f2: V_t·sin(δ) − Xq·Iq + Ra·Id = 0
```
where `I = S_pu / V_t` (constant kVA), `Id = I·sin(δ+φ)`, `Iq = I·cos(δ+φ)`.

Closes ~60–70% of the gap vs manufacturer published dip values. Remaining ~1–2% is dynamic saturation (not modeled). Apply 10–15% margin on acceptable dip thresholds for final decisions.

### Pre-event EMF (motor starting study)
Before the motor starts, pre-event E₀ is calculated from the actual base load at V_t = 1.0 pu using the same salient-pole equations. This correctly raises E₀ above 1.0 pu and shifts the initial load angle, giving a more realistic starting point for the post-disturbance solve.

## Reactance notes

| Symbol | Time period | Use for |
|--------|------------|---------|
| Xd" + Xq" | 0–50 ms (subtransient) | Initial peak dip, relay pickup, UPS hold-in |
| Xd' + Xq' | 50–500 ms (transient) | Motor acceleration window, protection coordination |
| Xd + Xq | >1 s (steady-state) | Theoretical open-loop floor; AVR prevents in practice |

Use **saturated** reactance values for load-step voltage dip analysis. Use **unsaturated** values for fault studies.

## License

MIT
