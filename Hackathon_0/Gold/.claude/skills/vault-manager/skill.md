# Vault Manager Skill
# Gold Tier - Task Processing & Workflow Management

Process tasks from Needs_Action, create plans, manage approvals, and move completed tasks to Done.

## Usage

```bash
/vault-manager
```

Or with specific action:
```bash
/vault-manager --action process
/vault-manager --action approve
/vault-manager --action cleanup
```

## What This Skill Does

1. **Scans `/Needs_Action/`** for pending tasks
2. **Analyzes each task** using Company_Handbook.md rules
3. **Creates execution plans** in `/Plans/`
4. **Generates approval requests** for sensitive actions
5. **Executes approved actions** via MCP servers
6. **Moves completed tasks** to `/Done/`
7. **Updates Dashboard** with current status

## Task Processing Workflow

### 1. Discovery
- Scan all subfolders in `/Needs_Action/`
- Prioritize by urgency (critical → high → medium → low)
- Group related tasks

### 2. Analysis
- Read task metadata and content
- Determine required actions
- Check approval requirements
- Estimate complexity

### 3. Planning
- Create detailed execution plan
- List all steps with checkboxes
- Identify dependencies
- Set completion criteria

### 4. Approval (if needed)
- Create approval request in `/Pending_Approval/`
- Include full context and reasoning
- Wait for human decision
- Process approved actions only

### 5. Execution
- Execute each step in plan
- Log all actions
- Handle errors gracefully
- Verify completion

### 6. Completion
- Move task to `/Done/`
- Update Dashboard
- Log metrics
- Archive related files

## Priority Handling

### Critical (Process Immediately)
- Keywords: "urgent", "asap", "emergency", "critical"
- Payment requests >$500
- Client escalations
- System errors

### High (Within 2 Hours)
- Client communications
- Pending approvals
- Important emails
- Social media opportunities

### Medium (Within 24 Hours)
- Routine emails
- File processing
- Regular updates
- Scheduled posts

### Low (When Available)
- Maintenance tasks
- Cleanup operations
- Non-urgent organization

## Approval Requirements

### Auto-Approve
- File organization
- Dashboard updates
- Log entries
- Reading/analyzing
- Creating drafts
- Moving files (except deletions)

### Requires Approval
- Sending emails to new contacts
- All payments >$50
- Social media posts
- Deletions
- External actions
- Sensitive communications
- Accounting transactions

## Plan Format

```markdown
---
type: execution_plan
task_id: TASK_12345
priority: high
created: 2026-02-28T20:00:00Z
status: in_progress
---

# Execution Plan: [Task Name]

## Objective
[Clear statement of what needs to be accomplished]

## Context
[Background information and relevant details]

## Steps
- [ ] Step 1: [Action description]
- [ ] Step 2: [Action description]
- [ ] Step 3: [Action description]

## Approval Required
- [ ] Email send to new contact (see /Pending_Approval/)
- [ ] Payment of $150 (see /Pending_Approval/)

## Completion Criteria
- All steps checked
- All approvals obtained
- Actions executed successfully
- Results verified

## Notes
[Any additional information or considerations]
```

## Ralph Wiggum Loop Integration

For complex multi-step tasks:
1. Create plan with clear completion criteria
2. Start Ralph Wiggum loop
3. Execute steps iteratively
4. Check completion after each iteration
5. Exit when task file in `/Done/`

## Error Handling

### Transient Errors
- Retry 3x with exponential backoff
- Log each attempt
- Continue to next step if successful

### Authentication Errors
- Alert human immediately
- Pause operations
- Wait for credential refresh

### Logic Errors
- Create escalation in `/Needs_Action/escalations/`
- Include full context
- Wait for human guidance

## Cleanup Actions

```bash
/vault-manager --action cleanup
```

Performs:
- Archive old logs (>90 days)
- Move stale tasks (>7 days in Needs_Action)
- Clean up empty folders
- Optimize file organization
- Update statistics

## Dashboard Updates

After processing, updates:
- Task counts by status
- Recent activity log
- Alerts and notifications
- Performance metrics
- System status

## Example Usage

```bash
# Process all pending tasks
/vault-manager

# Process only high priority tasks
/vault-manager --priority high

# Process specific domain
/vault-manager --domain email

# Cleanup and maintenance
/vault-manager --action cleanup
```

## Implementation Notes

This skill should:
1. Be the primary task processing engine
2. Follow Company_Handbook.md rules strictly
3. Create detailed, actionable plans
4. Respect approval workflows
5. Handle errors gracefully
6. Log all actions comprehensively
7. Update Dashboard in real-time
8. Support Ralph Wiggum loops for complex tasks

## Available Tools for Task Execution

### Facebook Comment Replies
**Tool**: `AI_Employee_Vault/mcp_servers/social_media/reply_facebook_comment.py`

**Purpose**: Securely reply to Facebook comments without exposing access tokens

**Usage**:
```bash
cd AI_Employee_Vault/mcp_servers/social_media
python reply_facebook_comment.py "<comment_id>" "<message>"
```

**Example**:
```bash
python reply_facebook_comment.py "122096860263074466_887671284268298" "Thanks for your comment!"
```

**When to Use**:
- Processing Facebook comment tasks from `/Needs_Action/facebook/`
- After approval is obtained for comment responses
- Keeps credentials secure (loaded from .env file)
- Uses MCP server's `reply_to_facebook_comment` function

**Security**: This tool uses the MCP server's secure credential management. Never expose access tokens in command-line arguments or logs.

### Instagram Comment Replies
**Tool**: `AI_Employee_Vault/mcp_servers/social_media/reply_instagram_comment.py`

**Purpose**: Securely reply to Instagram comments without exposing access tokens

**Usage**:
```bash
cd AI_Employee_Vault/mcp_servers/social_media
python reply_instagram_comment.py "<comment_id>" "<message>"
```

**Example**:
```bash
python reply_instagram_comment.py "17891269263305259" "Thank you for your comment! 🙏"
```

**When to Use**:
- Processing Instagram comment tasks from `/Needs_Action/instagram/`
- After approval is obtained for comment responses (if needed)
- Keeps credentials secure (loaded from .env file)
- Uses Instagram Graph API's comment reply endpoint

**How to Get Comment ID**:
1. Comment ID is included in the task file created by `social_media_watcher.py`
2. Look for the `id` field in the task metadata
3. Or fetch from Instagram API: `GET /{media_id}/comments`

**Security**: This tool uses the MCP server's secure credential management. Never expose access tokens in command-line arguments or logs.
