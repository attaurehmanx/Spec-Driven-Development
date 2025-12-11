---
id: 001
title: Add initial project constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution","initial","project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - None
---

## Prompt

Project:
Physical AI & Humanoid Robotics — AI-Native Textbook (Docusaurus) + Integrated RAG Chatbot.

Core Principles:
- High technical accuracy across ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA systems.
- Clear, consistent educational writing (grade 9–12).
- Code, diagrams, and explanations must reflect official tooling documentation.
- Production-ready RAG implementation using FastAPI, Neon Postgres, Qdrant, and OpenAI/ChatKit.

Key Standards:
- 12 chapters × 4 lessons (48 lessons total), each with definitions, examples, diagrams, exercises, and a quiz.
- Robotics content aligned with ROS 2 Humble + Isaac/NAV2 APIs.
- RAG chatbot must support full-book retrieval and “answer from selected text only.”
- Docusaurus v3-compatible structure, deployable to GitHub Pages.
- Python 3.10+ code, clean modular architecture, documented.

Constraints:
- Unified terminology across all chapters.
- All examples runnable in ROS 2 or simulation environments.
- Chatbot UI embedded directly into Docusaurus.
- All sections must remain concise, consistent, and technically traceable.

Success Criteria:
- Book builds cleanly with no broken links.
- Chatbot retrieves accurately, stores logs in Neon, and answers only from book data.
- Robotics modules match verified APIs and real system behavior.
- Capstone “Autonomous Humanoid” project is coherent and executable.

## Response snapshot

The project constitution has been updated to version 1.0.0. This is the initial creation of the constitution.

The following templates require manual review and updates to align with the new constitution:
*   `.specify/templates/plan-template.md`
*   `.specify/templates/spec-template.md`
*   `.specify/templates/tasks-template.md`

Suggested commit message: `docs: add initial project constitution v1.0.0`

## Outcome

- ✅ Impact: Project constitution defined and updated.
- 🧪 Tests: No tests were run for this task.
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: User needs to manually update templates.
- 🧠 Reflection: Successfully created initial project constitution and identified dependent template updates.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): Not applicable.
- Prompt variant (if applicable): None.
- Next experiment (smallest change to try): None.
