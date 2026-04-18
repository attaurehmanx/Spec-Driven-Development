---
name: ceo-briefing
description: Generate comprehensive weekly business briefing with financial analysis, task completion, and proactive suggestions
---

# CEO Briefing Generator - Weekly Business Audit

## Workflow

### Step 1: Gather Financial Data
```bash
# Navigate to accounting folder
cd AI_Employee_Vault/vault/Accounting

# Read monthly summaries
type *_Summary.md

# Check Odoo data if available
cd AI_Employee_Vault/mcp_servers/odoo
python server.py
```

### Step 2: Analyze Completed Tasks
```bash
# Navigate to Done folder
cd AI_Employee_Vault/vault/Done

# Count tasks completed this week
dir /s /b *.md | find /c ".md"

# Review recent completions
forfiles /p . /s /m *.md /d -7 /c "cmd /c echo @path"
```

### Step 3: Review Activity Logs
```bash
# Navigate to logs
cd AI_Employee_Vault/vault/Logs

# Read this week's logs
forfiles /p . /s /m *.json /d -7 /c "cmd /c type @path"
```

### Step 4: Check Social Media Performance
```bash
# Read social media log
cd AI_Employee_Vault/vault
type social_media_log.txt

# Check posted content
cd Posted
dir /s /b *.md
```

### Step 5: Read Business Goals
```bash
# Read current goals and metrics
cd AI_Employee_Vault/vault
type Business_Goals.md
```

### Step 6: Generate Briefing Document
Create file: `Briefings/YYYY-MM-DD_CEO_Briefing.md`

```markdown
---
type: ceo_briefing
period_start: [YYYY-MM-DD]
period_end: [YYYY-MM-DD]
generated: [ISO timestamp]
---

# Monday Morning CEO Briefing

**Period**: [Start Date] to [End Date]
**Generated**: [Timestamp]

## Executive Summary
[One paragraph overview with key wins and challenges]

## Financial Performance

| Metric | This Week | MTD | Target | Status |
|--------|-----------|-----|--------|--------|
| Revenue | $X | $X | $X | 🟢/🟡/🔴 |
| Expenses | $X | $X | $X | 🟢/🟡/🔴 |
| Net Profit | $X | $X | $X | 🟢/🟡/🔴 |

### Revenue Breakdown
- Invoiced: $X
- Received: $X
- Outstanding: $X

### Expense Breakdown
- By category (if available)

## Operational Performance

### Task Metrics
| Metric | Count |
|--------|-------|
| Tasks Completed | X |
| Pending Approval | X |
| In Progress | X |
| Overdue | X |

### Bottlenecks Identified
- [List tasks taking >2x expected time]

## Marketing & Engagement

### Social Media Metrics
| Platform | Posts | Engagement | Reach |
|----------|-------|------------|-------|
| LinkedIn | X | X | X |
| Facebook | X | X | X |
| Instagram | X | X | X |
| Twitter | X | X | X |

## Proactive Suggestions

### Cost Optimization
- [List opportunities]

### Process Improvements
- [List recommendations]

### Risk Alerts
- [List any concerns]

### Growth Opportunities
- [List opportunities]

## Upcoming Week

### Key Deadlines
- [Date]: [Deadline]

### Scheduled Activities
- [Activity]

### Resource Requirements
- [Requirements]

### Potential Blockers
- [Blockers]
```

## Analysis Criteria

### Financial Health Indicators
| Indicator | Green | Yellow | Red |
|-----------|-------|--------|-----|
| Revenue vs Target | >90% | 70-90% | <70% |
| Expense Ratio | <40% | 40-50% | >50% |
| Cash Flow | Positive | Break-even | Negative |

### Task Performance Indicators
| Indicator | Green | Yellow | Red |
|-----------|-------|--------|-----|
| Completion Rate | >95% | 85-95% | <85% |
| On-Time Delivery | >90% | 80-90% | <80% |
| Approval Rate | >90% | 80-90% | <80% |

## Commands Reference

| Action | Command |
|--------|---------|
| Generate weekly briefing | Run skill without args |
| Specific date range | `--start YYYY-MM-DD --end YYYY-MM-DD` |
| Financial only | `--section financial` |
| Operations only | `--section operations` |

## Output Location
`AI_Employee_Vault/vault/Briefings/YYYY-MM-DD_CEO_Briefing.md`
