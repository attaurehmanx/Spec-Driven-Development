# email-sender

Manages email drafting and sending through MCP server integration.

## Usage

```
/email-sender
```

Then describe what you need:
- "Draft a reply to [email]"
- "Send email to [recipient] about [topic]"
- "Review pending email drafts"

## What This Skill Does

1. **Email Drafting**: Creates professional email drafts
2. **Reply Generation**: Drafts replies to incoming emails
3. **Approval Workflow**: Requires approval for sending emails
4. **MCP Integration**: Uses email MCP server for actual sending
5. **Logging**: Records all email activity

## Email Types

### Reply to Incoming Email
```
/email-sender
Draft a reply to the email in EMAIL_abc123.md in Needs_Action
```

### New Email
```
/email-sender
Draft an email to client@example.com about project status update
```

### Follow-up Email
```
/email-sender
Create a follow-up email for the proposal sent last week
```

## Approval Workflow

1. Skill creates draft in Pending_Approval/
2. Human reviews draft
3. Human moves to Approved/ to send
4. MCP server sends email
5. Confirmation logged
6. File moved to Done/

## Email Structure

```markdown
---
type: email_draft
to: recipient@example.com
subject: Email Subject
priority: medium
requires_approval: true
---

## Email Draft

**To**: recipient@example.com
**Subject**: Email Subject
**Type**: Reply / New / Follow-up

### Email Body

[Draft content here]

---

## Approval
- [ ] Content reviewed
- [ ] Tone appropriate
- [ ] No errors
- [ ] Ready to send

Move to Approved/ folder to send.
```

## Best Practices

### Professional Tone
- Clear and concise
- Friendly but professional
- Proper grammar and spelling
- Appropriate greeting and closing

### Structure
1. Greeting
2. Context/Reference
3. Main message
4. Call to action
5. Closing

### Response Times
- Urgent: Within 2 hours
- Important: Same day
- Regular: Within 24 hours

## Auto-Approve Rules

Based on Company_Handbook.md, some emails may be auto-approved:
- Replies to known contacts
- Standard acknowledgments
- Routine updates

All other emails require human approval.

## MCP Server Setup

The email-sender skill requires an MCP server for Gmail integration.

### Configuration
```json
{
  "servers": [
    {
      "name": "email",
      "command": "node",
      "args": ["/path/to/email-mcp/index.js"],
      "env": {
        "GMAIL_CREDENTIALS": "/path/to/credentials.json"
      }
    }
  ]
}
```

### Required Permissions
- Gmail API access
- Send email permission
- Read email permission

## Examples

### Reply to Client Email
```
/email-sender
Draft a professional reply to the client email in Needs_Action. Thank them for their inquiry and provide the requested information about our services.
```

### Send Invoice
```
/email-sender
Create an email to send the invoice in Inbox/invoice.pdf to client@example.com
```

### Follow-up
```
/email-sender
Draft a polite follow-up email for the proposal sent 3 days ago
```
