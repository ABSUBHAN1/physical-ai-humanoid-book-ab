---
id: 1
title: create-project-constitution
stage: constitution
date: 2026-01-06
surface: agent
model: Qwen
feature: none
branch: main
user: user
command: /sp.constitution
labels: [constitution, textbook, physical-ai, humanoid-robotics]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:
---

## Prompt

---
description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync.
handoffs: 
  - label: Build Specification
    agent: sp.specify
    prompt: Implement the feature specification based on the updated constitution. I want to build...
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

Follow this execution flow:

1. Load the existing constitution template at `.specify/memory/constitution.md`.
   - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
   **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

2. Collect/derive values for placeholders:
   - If user input (conversation) supplies a value, use it.
   - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
   - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
   - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
     - MAJOR: Backward incompatible governance/principle removals or redefinitions.
     - MINOR: New principle/section added or materially expanded guidance.
     - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
   - If version bump type ambiguous, propose reasoning before finalizing.

3. Draft the updated constitution content:
   - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
   - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
   - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
   - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

4. Consistency propagation checklist (convert prior checklist into active validations):
   - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
   - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
   - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
   - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
   - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
   - Version change: old → new
   - List of modified principles (old title → new title if renamed)
   - Added sections
   - Removed sections
   - Templates requiring updates (✅ updated / ⚠ pending) with file paths
   - Follow-up TODOs if any placeholders intentionally deferred.

6. Validation before final output:
   - No remaining unexplained bracket tokens.
   - Version line matches report.
   - Dates ISO format YYYY-MM-DD.
   - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

8. Output a final summary to the user with:
   - New version and bump rationale.
   - Any files flagged for manual follow-up.
   - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

Formatting & Style Requirements:

- Use Markdown headings exactly as in the template (do not demote/promote levels).
- Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
- Keep a single blank line between sections.
- Avoid trailing whitespace.

If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.


/sp.constitution # =========================================
# sp.constitution.toml
# 📘 Spec Constitution: Physical AI & Humanoid Robotics Textbook
# =========================================

description = "AI-native, university-level textbook: Physical AI & Humanoid Robotics, from digital intelligence to embodied agents."

prompt = """
---
# Mission
Create an AI-native, university-level textbook titled:

“Physical AI & Humanoid Robotics: From Digital Intelligence to Embodied Agents”

The book must teach students how to design, simulate, and control humanoid robots using modern AI systems, robotics middleware, physics simulators, and large language models.

Requirements:
- Technically rigorous
- Beginner-friendly but industry-relevant
- Designed for AI-assisted learning (RAG, personalization, translation)
- Ready for publication using Docusaurus and GitHub Pages

---

# Target Audience
- AI & Software Engineering students
- Robotics beginners transitioning from pure AI
- Developers familiar with Python, ML, or Web but new to robotics
- Learners from PIAIC, GIAIC, Panaversity, etc.

Assume:
- Basic Python knowledge
- No prior robotics experience
- Curiosity about humanoid robots and physical intelligence

---

# Learning Philosophy
Embodied Intelligence First:

- Intelligence emerges from interaction with the physical world
- Robots are AI agents with bodies, sensors, and actuators
- Simulation precedes real hardware (Sim-to-Real)
- Language models act as cognitive planners, not controllers

Each chapter must:
- Explain concepts intuitively
- Ground theory in physical reality
- Include diagrams (textual descriptions)
- Include hands-on labs and experiments
- Prepare content for RAG-based questioning

---

# Book Structure Rules

## Modular Design
- One folder per module
- One markdown file per chapter
- Front-matter metadata for indexing
- Chunk content for vector embeddings

## Chapter Format
Each chapter must include:
1. Chapter Overview
2. Why This Matters in Physical AI
3. Core Concepts
4. System Architecture or Mental Model
5. Hands-On Lab / Simulation
6. Code Examples (Python / ROS 2)
7. Common Mistakes & Debugging
8. Industry & Research Context
9. Review Questions
10. Glossary

---

# Technical Stack Alignment
- ROS 2 (rclpy, Nav2)
- Gazebo & Unity
- NVIDIA Isaac Sim & Isaac ROS
- OpenAI Whisper
- LLM-based planners (GPT-style)
- Vision-Language-Action (VLA) systems

Avoid deprecated tools.
Prefer modern, production-ready workflows.

---

# AI-Native Features

## RAG Optimization
Content must be:
- Self-contained per section
- Concise and explicit
- Suitable for semantic search and retrieval

## Personalization Hooks
Each chapter must support:
- Beginner / Intermediate / Advanced explanations
- Hardware-aware suggestions
- Software background adaptation

Include markers like:
> *If you are new to robotics...*  
> *If you have ML experience...*

---

# Language & Style Guide
- Clear, confident, mentor-like tone
- Explain before formalizing
- Analogies from human biology
- Bullet points, diagrams, tables

Primary language: English  
Secondary support: Urdu (simple, educational)

---

# Safety & Ethics
Include:
- Robot safety
- Simulation-first approach
- Ethical humanoid AI
- Human-centered AI principles

Avoid:
- Military robotics
- Surveillance misuse narratives

---

# Capstone Alignment
Prepare learners for **Autonomous Humanoid Capstone**:
- Voice command → LLM planning
- ROS 2 action execution
- Navigation + perception
- Object interaction
- Conversational feedback

---

# Quality Bar
Content should be:
- Comparable to top AI/Robotics programs
- Better structured than typical robotics books
- AI-assisted consumption ready
- Suitable for Panaversity publication

Claude Code must:
- Think step-by-step
- Verify technical correctness
- Avoid hallucinated APIs
- Prefer official documentation logic

---

# Success Definition
Book is successful if:
- Student can build a simulated humanoid
- RAG chatbot answers accurately
- Content scales beginner → advanced
- Book feels alive, not static

---

# Final Instruction to AI
You are not just writing documentation.

You are:
- Designing a learning operating system
- Teaching future builders of humanoid intelligence
- Bridging software minds with physical bodies

Write responsibly, clearly, and with vision.
"""

## Response snapshot

I have successfully updated the project constitution for the Physical AI & Humanoid Robotics textbook based on the provided principles. Here's the summary:

**New Version**: 1.0.0 (Initial version for this project)
**Bump Rationale**: This is the initial constitution for the Physical AI & Humanoid Robotics textbook project, establishing the foundational principles and governance structure.

**Files Updated**:
1. `.specify/memory/constitution.md` - Main constitution file with 6 core principles
2. `.specify/templates/plan-template.md` - Updated constitution check section
3. `.specify/templates/spec-template.md` - Updated functional requirements section
4. `.specify/templates/tasks-template.md` - Updated all phases to align with textbook requirements

**Key Principles Established**:
1. Embodied Intelligence First
2. Technical Rigor with Accessibility
3. AI-Assisted Learning
4. Modular Design
5. Hands-On Learning
6. Safety & Ethics

**Suggested Commit Message**: 
`docs: create constitution for Physical AI & Humanoid Robotics textbook v1.0.0`

All templates have been updated to align with the new constitution, ensuring that future development will follow the established principles for the textbook project.

## Outcome

- ✅ Impact: High (new constitution established)
- 🧪 Tests: No tests added
- 📁 Files: 4 files updated
- 🔁 Next prompts: None
- 🧠 Reflection: Constitution successfully created for Physical AI & Humanoid Robotics textbook

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): None