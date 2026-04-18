# 📖 The Ultimate Guide: How to Do Everything in the AI Employee Project

Welcome to your AI Employee - Silver Tier. This guide is a single, comprehensive resource that explains the entire system from setup to daily operations.

---

## 📑 Table of Contents
1. [Architecture Overview](#1-architecture-overview)
2. [Prerequisites & Installation](#2-prerequisites--installation)
3. [Watcher Configuration](#3-watcher-configuration)
4. [Running the System](#4-running-the-system)
5. [The Vault Workflow](#5-the-vault-workflow)
6. [Agent Skills (Claude Integration)](#6-agent-skills-claude-integration)
7. [Manual Actions & Approvals](#7-manual-actions--approvals)
8. [Daily Maintenance & Routines](#8-daily-maintenance--routines)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. Architecture Overview

Your AI Employee is a **multi-agent, local-first automation system** that follows this cycle:
1.  **Watchers**: Python scripts monitor external sources (Files, Gmail, WhatsApp, LinkedIn).
2.  **Vault**: Items are detected and placed in the `Inbox/` and `Needs_Action/` folders of an Obsidian vault.
3.  **Claude (The Brain)**: You use Claude Code with specialized "Agent Skills" to process these tasks.
4.  **Action**: Claude creates plans, drafts responses, and moves items to `Pending_Approval/`.
5.  **Human**: You review, approve, or reject the proposed actions.
6.  **Execution**: Approved emails are sent automatically; LinkedIn posts are ready for you to copy-paste.

---

## 2. Prerequisites & Installation

### Required Software
- **Python 3.13+** (Required)
- **Claude Code** (For the reasoning loop)
- **Obsidian** (To view the dashboard and manage tasks)
- **Git** (For version control)

### Step-by-Step Installation (Windows)
1.  Open a terminal in the project root.
2.  Run `cd AI_Employee_Vault && setup.bat`.
3.  This script will:
    - Install all dependencies (`pip install -r requirements.txt`).
    - Create the folder structure for your Obsidian vault.
    - Set up the `Drop_Folder/` in the root directory.
    - Install Playwright (for WhatsApp monitoring).

### Step-by-Step Installation (Linux/Mac)
1.  Open a terminal in the project root.
2.  Run `cd AI_Employee_Vault && chmod +x setup.sh && ./setup.sh`.

---

## 3. Watcher Configuration

Each watcher runs as a separate process managed by the **Orchestrator**.

### 📁 File System Watcher
- **Monitor**: `Drop_Folder/`
- **Setup**: Automatic. Just drop a file there, and it appears in the vault.

### 📧 Gmail Watcher
- **Monitor**: Unread emails.
- **Setup**:
    1.  Enable Gmail API in [Google Cloud Console](https://console.cloud.google.com/).
    2.  Download `credentials.json` and place it in `AI_Employee_Vault/`.
    3.  The first time you run it, a browser window will open for OAuth2 authentication.

### 💬 WhatsApp Watcher
- **Monitor**: WhatsApp Web for unread messages.
- **Setup**:
    1.  Ensure Playwright is installed (`python -m playwright install chromium`).
    2.  Run `python whatsapp_watcher_simple.py` individually first.
    3.  Scan the QR code with your phone. The session is saved in `whatsapp_session/`.

### 💼 LinkedIn Watcher
- **Monitor**: Schedule-based (every 24 hours).
- **Setup**: Automatic. It creates a "Daily LinkedIn Post" task in `Needs_Action/linkedin/`.

---

## 4. Running the System

To start the entire "employee" in the background:

```bash
cd AI_Employee_Vault
python orchestrator.py
```

The Orchestrator will start all enabled watchers and monitor their health. If a watcher crashes, it will attempt to restart it.

---

## 5. The Vault Workflow

The Obsidian vault is the "office" where work happens.

| Folder | Purpose |
| :--- | :--- |
| **`Inbox/`** | Raw files or attachments from watchers. |
| **`Needs_Action/`** | Markdown task files (e.g., `FILE_invoice.md`, `EMAIL_123.md`). |
| **`Plans/`** | Claude creates a `PLAN_*.md` here for complex tasks. |
| **`Pending_Approval/`** | Drafts or actions awaiting your review. |
| **`Approved/`** | Move items here to authorize execution. |
| **`Rejected/`** | Move items here to cancel them. |
| **`Done/`** | History of all completed tasks. |
| **`Logs/`** | JSON-formatted audit trail of every action. |
| **`Dashboard.md`** | Your real-time status report (open this in Obsidian!). |

---

## 6. Agent Skills (Claude Integration)

When you run `claude` inside `AI_Employee_Vault/`, you have access to 4 specialized skills:

### 🛠️ `/vault-manager`
Processes everything in the vault.
- **Example**: "Check the vault and process pending tasks."
- **Action**: It reads `Needs_Action/`, creates plans, and moves items to `Pending_Approval/`.

### 📰 `/daily-briefing`
Summarizes your day.
- **Example**: "Generate today's briefing."
- **Action**: Creates a report in `Briefings/` with metrics and top priorities.

### 💼 `/linkedin-poster`
Drafts social media content.
- **Example**: "Draft a post about our new AI automation system."
- **Action**: Creates a post draft in `LinkedIn_Drafts/` and an approval request.

### 📧 `/email-sender`
Drafts and manages emails.
- **Example**: "Draft a reply to the email from John Smith."
- **Action**: Creates an `APPROVAL_EMAIL_*.md` file in `Pending_Approval/`.

---

## 7. Manual Actions & Approvals

The AI Employee **never** sends an email or posts to LinkedIn without your approval.

### How to Approve an Email:
1.  Open Obsidian and check `Pending_Approval/gmail/`.
2.  Review the drafted email (To, Subject, Body).
3.  If you like it, **move the file** to `Approved/gmail/`.
4.  The `email_sender` watcher (part of the orchestrator) will detect it, send it via Gmail API, and move the file to `Done/gmail/`.

### How to Approve a LinkedIn Post:
1.  Check `Pending_Approval/linkedin/`.
2.  Review the draft.
3.  Move it to `Approved/linkedin/`.
4.  Since there is no "Direct Post" API (to avoid bans), you then copy the text and post it manually. Move to `Done/linkedin/` when finished.

---

## 8. Daily Maintenance & Routines

Follow this daily routine for maximum efficiency:

- **Morning (8:00 AM)**:
    - Run `/daily-briefing` to see what arrived overnight.
    - Check the `Dashboard.md` for urgent alerts.
- **Midday (12:00 PM)**:
    - Run `/vault-manager` to process any new file drops or emails.
    - Review `Pending_Approval/` and move files to `Approved/`.
- **Evening (6:00 PM)**:
    - Final check of the `Done/` folder to ensure everything went through.
    - Review `Logs/` if anything seems incorrect.

---

## 9. Troubleshooting

**"Watchers aren't detecting files"**
- Ensure `orchestrator.py` is running in a terminal.
- Check `orchestrator.log` for error messages.
- Verify you are dropping files into the root `Drop_Folder/`, not inside the vault.

**"Gmail authentication failed"**
- Delete `token.json` and `token_sender.json` in `AI_Employee_Vault/`.
- Run `python gmail_watcher.py` to re-authenticate.

**"WhatsApp window is white or stuck"**
- Close the WhatsApp browser window.
- Ensure your internet connection is stable.
- The `whatsapp_watcher_simple.py` script will attempt to reconnect.

**"Claude says it can't find a skill"**
- Ensure you are running `claude` from the `AI_Employee_Vault/` directory.
- Check that `.claude/skills/` exists and contains the skill folders.

---

*This guide is part of the Personal AI Employee Hackathon - Silver Tier Implementation.*
*Last Updated: 2026-04-18*
