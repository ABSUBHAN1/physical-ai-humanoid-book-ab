# Data Model: AI Textbook Platform

## Core Entities

### User
- `id`: UUID (primary key)
- `email`: String (unique)
- `name`: String
- `learning_preferences`: JSON (personalization settings)
- `language_preference`: String (default: "en", options: ["en", "ur"])
- `technical_background`: String (options: ["beginner", "intermediate", "advanced"])
- `progress`: JSON (tracking module/chapter completion)
- `created_at`: DateTime
- `updated_at`: DateTime

### Module
- `id`: UUID (primary key)
- `title`: String
- `description`: Text
- `order`: Integer (sequence in curriculum)
- `learning_objectives`: Array of strings
- `prerequisites`: Array of Module IDs
- `created_at`: DateTime
- `updated_at`: DateTime

### Chapter
- `id`: UUID (primary key)
- `module_id`: UUID (foreign key to Module)
- `title`: String
- `content`: Text (Markdown format)
- `order`: Integer (sequence in module)
- `learning_objectives`: Array of strings
- `estimated_duration`: Integer (minutes)
- `hands_on_lab`: Text (Markdown format)
- `code_examples`: JSON (structured code examples)
- `created_at`: DateTime
- `updated_at`: DateTime

### ChapterTranslation
- `id`: UUID (primary key)
- `chapter_id`: UUID (foreign key to Chapter)
- `language`: String (e.g., "ur", "en")
- `title`: String
- `content`: Text (Markdown format)
- `hands_on_lab`: Text (Markdown format)
- `created_at`: DateTime
- `updated_at`: DateTime

### LabExercise
- `id`: UUID (primary key)
- `chapter_id`: UUID (foreign key to Chapter)
- `title`: String
- `description`: Text
- `simulation_environment`: String (options: ["gazebo", "unity", "nvidia-isaac", "none"])
- `code_template`: Text
- `solution`: Text
- `created_at`: DateTime
- `updated_at`: DateTime

### UserProgress
- `id`: UUID (primary key)
- `user_id`: UUID (foreign key to User)
- `chapter_id`: UUID (foreign key to Chapter)
- `status`: String (options: ["not-started", "in-progress", "completed"])
- `completion_percentage`: Integer
- `time_spent`: Integer (seconds)
- `last_accessed`: DateTime
- `created_at`: DateTime
- `updated_at`: DateTime

### ChatSession
- `id`: UUID (primary key)
- `user_id`: UUID (foreign key to User)
- `module_id`: UUID (foreign key to Module, optional)
- `chapter_id`: UUID (foreign key to Chapter, optional)
- `created_at`: DateTime
- `updated_at`: DateTime

### ChatMessage
- `id`: UUID (primary key)
- `session_id`: UUID (foreign key to ChatSession)
- `role`: String (options: ["user", "assistant"])
- `content`: Text
- `timestamp`: DateTime
- `context_chunks`: Array of strings (relevant content chunks)

### Embedding
- `id`: UUID (primary key)
- `content_id`: String (ID of the content being embedded)
- `content_type`: String (options: ["chapter", "lab_exercise", "module"])
- `chunk_text`: Text (the text that was embedded)
- `chunk_index`: Integer (position of chunk in original content)
- `embedding_vector`: Array of floats (the actual embedding)
- `language`: String (language of the chunk)
- `created_at`: DateTime

## Relationships

- User has many UserProgress entries
- Module has many Chapters
- Chapter belongs to one Module
- Chapter has many LabExercises
- Chapter has many ChapterTranslations
- UserProgress links Users to Chapters
- ChatSession links Users to their chat sessions
- ChatMessage belongs to ChatSession
- Embedding represents vector embeddings of content

## Validation Rules

1. **User**: Email must be valid and unique
2. **Module**: Order values must be unique within curriculum
3. **Chapter**: Order values must be unique within module
4. **ChapterTranslation**: Only one translation per language per chapter
5. **UserProgress**: Only one progress record per user-chapter combination
6. **Embedding**: Content ID and type combination must be unique for each language

## State Transitions

### UserProgress States
- `not-started` → `in-progress` (when user starts chapter)
- `in-progress` → `completed` (when user completes chapter)
- `completed` → `in-progress` (if user wants to revisit)

### Technical Background Options
- `beginner`: Basic Python knowledge, no robotics experience
- `intermediate`: Some robotics experience, familiar with basic concepts
- `advanced`: Deep robotics experience, familiar with advanced concepts