# Analysis Scripts

Scripts for analyzing experiment logs produced by `logger_node`.

## Prerequisites

Install required Python packages:

```bash
pip install pandas matplotlib
```

## Scripts

### `analyze_run.py`

Analyzes a single experiment run CSV and generates metrics and plots.

**Usage**:
```bash
python3 scripts/analyze_run.py \
  --csv ~/alc_logs/20260114_220113.csv \
  --outdir results \
  --save_plots
```

**Outputs**:
- `{run_name}_summary.csv`: One-row summary with metrics
- `{run_name}_reward.png`: Reward curve over steps
- `{run_name}_cum_reward.png`: Cumulative reward curve
- `{run_name}_knowledge.png`: Knowledge learning curve

**Metrics Computed**:
- `learning_gain`: Final knowledge - initial knowledge
- `overload_rate`: Fraction of time cognitive load > threshold
- `success_rate`: Fraction of correct responses
- `accessibility_match_rate`: Fraction of actions with modality-disability match
- `mean_reward`: Average reward per step
- `final_knowledge`: Knowledge level at end of run
- `n_steps`: Total number of steps

---

### `analyze_batch.py`

Analyzes multiple CSV files and generates an aggregated summary table.

**Usage**:
```bash
python3 scripts/analyze_batch.py \
  --glob "~/alc_logs/*.csv" \
  --out results/summary_all.csv
```

**Outputs**:
- `summary_all.csv`: Table with one row per run, including all metrics
- Prints aggregated statistics grouped by condition and disability profile

**Metrics Computed** (same as `analyze_run.py`):
- `learning_gain`: Final knowledge - initial knowledge
- `final_knowledge`: Knowledge level at end of run
- `overload_rate`: Fraction of time cognitive load > threshold
- `success_rate`: Fraction of correct responses
- `accessibility_match_rate`: Fraction of actions with modality-disability match
- `mean_reward`: Average reward per step
- `n_steps`: Total number of steps

**Useful for**: Comparing multiple runs across different conditions.

---

### `generate_table_5_1.py`

Generates Table 5.1: Mean final knowledge (± SD) comparing Baseline (fixed) and Adaptive tutors across learner profiles.

**Usage**:
```bash
python3 scripts/generate_table_5_1.py \
  --csv results/summary_all.csv \
  --out results/table_5_1.csv
```

**Prerequisites**:
- Must run `analyze_batch.py` first to generate `summary_all.csv`
- Batch summary must include runs with `condition` in `["fixed", "adaptive"]`
- Batch summary must include `final_knowledge` column

**Outputs**:
- `table_5_1.csv`: Formatted table with disability profiles as rows, conditions as columns
- Console output: Formatted table and summary statistics

**Table Format**:
```
Disability Profile | Baseline (fixed) | Adaptive
none               | 0.75 ± 0.12     | 0.82 ± 0.10
dyslexia           | 0.68 ± 0.15     | 0.79 ± 0.11
...
```

**Useful for**: Chapter 5 results tables, comparing baseline vs adaptive policies.

---

## Example Workflow

1. **Run experiments**:
   ```bash
   ros2 launch alc_bringup full_system.launch.py \
     policy_mode:=fixed \
     disability_profile:=dyslexia \
     seed:=42
   ```

2. **Analyze single run**:
   ```bash
   python3 scripts/analyze_run.py \
     --csv ~/alc_logs/20260114_220113.csv \
     --outdir results \
     --save_plots
   ```

3. **Analyze all runs**:
   ```bash
   python3 scripts/analyze_batch.py \
     --glob "~/alc_logs/*.csv" \
     --out results/summary_all.csv
   ```

4. **Generate Table 5.1**:
   ```bash
   python3 scripts/generate_table_5_1.py \
     --csv results/summary_all.csv \
     --out results/table_5_1.csv
   ```

5. **View results**:
   - Check `results/` directory for plots and summary CSVs
   - Use summary CSV for statistical analysis
   - Use `table_5_1.csv` for publication-ready tables

---

## Integration with Dissertation

These scripts support:
- **Chapter 5**: Metrics computation and visualization
- **Reproducibility**: Clear pipeline from logs → metrics → plots
- **Statistical Analysis**: Summary tables for condition comparisons
