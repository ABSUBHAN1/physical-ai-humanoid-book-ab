<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections (new constitution)
Removed sections: N/A
Templates requiring updates: ✅ updated - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Constitution

## Core Principles

### Embodied Intelligence First
Intelligence emerges from interaction with the physical world; Robots are AI agents with bodies, sensors, and actuators; Simulation precedes real hardware (Sim-to-Real); Language models act as cognitive planners, not controllers. This principle ensures that all content emphasizes the connection between physical reality and AI systems.

### Technical Rigor with Accessibility
All content must be technically rigorous yet beginner-friendly and industry-relevant. We assume basic Python knowledge but no prior robotics experience. This ensures the material is approachable while maintaining professional standards.

### AI-Assisted Learning
The textbook is designed for AI-assisted learning (RAG, personalization, translation) with content optimized for RAG-based questioning. This enables adaptive learning experiences and accessibility across different languages and learning styles.

### Modular Design
One folder per module, one markdown file per chapter, front-matter metadata for indexing, chunk content for vector embeddings. This structure enables efficient content management and retrieval for both human readers and AI systems.

### Hands-On Learning
Each chapter must include hands-on labs and experiments, code examples (Python/ROS 2), common mistakes & debugging sections. This ensures students gain practical experience with the concepts being taught.

### Safety & Ethics
Include robot safety, simulation-first approach, ethical humanoid AI, human-centered AI principles. This ensures responsible development of humanoid robotics technology with appropriate safety considerations.

## Technical Stack Alignment
Content must align with modern, production-ready tools: ROS 2 (rclpy, Nav2), Gazebo & Unity, NVIDIA Isaac Sim & Isaac ROS, OpenAI Whisper, LLM-based planners (GPT-style), Vision-Language-Action (VLA) systems. Avoid deprecated tools and prefer workflows that reflect current industry practices.

## Development Workflow
Content should be comparable to top AI/Robotics programs, better structured than typical robotics books, and AI-assisted consumption ready. Each chapter must follow the required 10-section format: Chapter Overview, Why This Matters in Physical AI, Core Concepts, System Architecture or Mental Model, Hands-On Lab/Simulation, Code Examples, Common Mistakes & Debugging, Industry & Research Context, Review Questions, and Glossary.

## Governance
All content must follow the specified chapter format with 10 required sections, use clear and confident mentor-like tone, and support personalization hooks for different experience levels. The constitution supersedes all other practices and must be referenced during content creation and review. Amendments require documentation of the change, approval process, and migration plan for existing content.

**Version**: 1.0.0 | **Ratified**: 2026-01-06 | **Last Amended**: 2026-01-06