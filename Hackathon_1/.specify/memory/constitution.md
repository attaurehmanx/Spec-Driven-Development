<!-- Sync Impact Report:
Version change: None → 1.0.0
Modified principles:
  - None (new constitution)
Added sections:
  - Key Standards
  - Constraints
Removed sections:
  - None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ✅ updated (no files found)
Follow-up TODOs:
  - None
-->
# Physical AI & Humanoid Robotics — AI-Native Textbook (Docusaurus) + Integrated RAG Chatbot. Constitution

## Core Principles

### I. High Technical Accuracy
High technical accuracy across ROS 2, Gazebo, Unity, NVIDIA Isaac, VLA systems.

### II. Clear, Consistent Educational Writing
Clear, consistent educational writing (grade 9–12).

### III. Official Tooling Documentation Reflection
Code, diagrams, and explanations must reflect official tooling documentation.

### IV. Production-Ready RAG Implementation
Production-ready RAG implementation using FastAPI, Neon Postgres, Qdrant, and OpenAI/ChatKit.

## Key Standards

- 12 chapters × 4 lessons (48 lessons total), each with definitions, examples, diagrams, exercises, and a quiz.
- Robotics content aligned with ROS 2 Humble + Isaac/NAV2 APIs.
- RAG chatbot must support full-book retrieval and “answer from selected text only.”
- Docusaurus v3-compatible structure, deployable to GitHub Pages.
- Python 3.10+ code, clean modular architecture, documented.

## Constraints

- Unified terminology across all chapters.
- All examples runnable in ROS 2 or simulation environments.
- Chatbot UI embedded directly into Docusaurus.
- All sections must remain concise, consistent, and technically traceable.

## Governance

This constitution supersedes all other practices; Amendments require documentation, approval, and a migration plan.

**Success Criteria:**
- Book builds cleanly with no broken links.
- Chatbot retrieves accurately, stores logs in Neon, and answers only from book data.
- Robotics modules match verified APIs and real system behavior.
- Capstone “Autonomous Humanoid” project is coherent and executable.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
