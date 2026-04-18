# CEO Briefing Generator Skill
# Gold Tier - Weekly Business Audit

Generate comprehensive weekly business briefing with financial analysis, task completion, and proactive suggestions.

## Usage

```bash
/ceo-briefing
```

## What This Skill Does

1. **Analyzes the past week** of business activity
2. **Reviews financial data** from Odoo accounting
3. **Summarizes completed tasks** and identifies bottlenecks
4. **Audits subscriptions** for cost optimization
5. **Analyzes social media** performance
6. **Generates proactive suggestions** for improvements
7. **Creates Monday Morning CEO Briefing** in `/Briefings/`

## Requirements

- Access to `/Accounting/` folder with transaction data
- Access to `/Done/` folder with completed tasks
- Access to `/Logs/` folder with activity logs
- Business_Goals.md with targets and metrics
- Company_Handbook.md with business rules

## Output

Creates a comprehensive briefing file:
`/Briefings/YYYY-MM-DD_CEO_Briefing.md`

## Briefing Sections

### 1. Executive Summary
- One paragraph overview
- Key wins and challenges
- Overall health indicator

### 2. Financial Performance
- Revenue (week, MTD, vs target)
- Expenses by category
- Net profit/loss
- Cash flow status
- Outstanding invoices
- Upcoming payments

### 3. Operational Performance
- Tasks completed vs planned
- Bottlenecks (tasks >2x expected time)
- Client interactions
- Project status updates

### 4. Marketing & Engagement
- Social media metrics (posts, engagement, reach)
- Email campaign performance
- Lead generation
- Content performance

### 5. Proactive Suggestions
- Cost optimization opportunities
- Process improvements
- Risk alerts
- Growth opportunities

### 6. Upcoming Week
- Key deadlines
- Scheduled activities
- Resource requirements
- Potential blockers

## Example Briefing

```markdown
# Monday Morning CEO Briefing
**Period**: 2026-02-21 to 2026-02-28
**Generated**: 2026-02-28 20:00:00

## Executive Summary
Strong week with steady progress on Gold tier implementation.
System infrastructure complete, ready for production testing.

## Financial Performance
- **This Week**: $0 (system setup phase)
- **MTD**: $0
- **Target**: $10,000/month
- **Status**: On track for Q1 launch

## Operational Performance
- ✅ 15 tasks completed
- ⚠️ 2 bottlenecks identified
- 📊 8 watchers operational
- 🎯 Gold tier 90% complete

## Proactive Suggestions
1. **Test Odoo Integration**: Schedule test transactions
2. **Social Media Launch**: Prepare first week of content
3. **Client Outreach**: Begin prospecting for first clients

## Upcoming Week
- Complete Gold tier documentation
- Record demo video
- Submit hackathon entry
- Begin Platinum tier planning
```

## Implementation Notes

This skill should:
1. Read all relevant data files
2. Parse and analyze metrics
3. Identify patterns and anomalies
4. Generate actionable insights
5. Format as professional briefing
6. Save to Briefings folder
7. Update Dashboard with summary
