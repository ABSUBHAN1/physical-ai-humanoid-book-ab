# API Contract: User & Personalization Service

## Overview
The User & Personalization service manages user profiles, preferences, and learning progress. It enables personalized learning experiences based on user background and progress.

## Base URL
`/api/v1/user`

## Endpoints

### User registration
- **POST** `/register`
- **Description**: Register a new user account
- **Authentication**: Not required
- **Request Body**:
  ```json
  {
    "email": "string (valid email)",
    "password": "string (min 8 characters)",
    "name": "string",
    "technical_background": "string (beginner|intermediate|advanced)",
    "language_preference": "string (en|ur)"
  }
  ```
- **Response**:
  - 201 Created
  ```json
  {
    "user_id": "string",
    "email": "string",
    "name": "string",
    "access_token": "string",
    "refresh_token": "string"
  }
  ```

### User login
- **POST** `/login`
- **Description**: Authenticate user and return tokens
- **Authentication**: Not required
- **Request Body**:
  ```json
  {
    "email": "string",
    "password": "string"
  }
  ```
- **Response**:
  - 200 OK
  ```json
  {
    "user_id": "string",
    "email": "string",
    "name": "string",
    "access_token": "string",
    "refresh_token": "string"
  }
  ```

### Get user profile
- **GET** `/profile`
- **Description**: Retrieve user profile and preferences
- **Authentication**: Required (JWT token)
- **Response**:
  - 200 OK
  ```json
  {
    "user_id": "string",
    "email": "string",
    "name": "string",
    "technical_background": "string (beginner|intermediate|advanced)",
    "language_preference": "string (en|ur)",
    "learning_preferences": {
      "difficulty_level": "string",
      "learning_pace": "string (slow|moderate|fast)",
      "preferred_formats": ["string (text|video|interactive)"]
    },
    "progress_summary": {
      "completed_modules": "integer",
      "total_modules": "integer",
      "completed_chapters": "integer",
      "total_chapters": "integer",
      "total_learning_time": "integer (minutes)"
    }
  }
  ```

### Update user preferences
- **PUT** `/preferences`
- **Description**: Update user preferences for personalization
- **Authentication**: Required (JWT token)
- **Request Body**:
  ```json
  {
    "technical_background": "string (beginner|intermediate|advanced)",
    "language_preference": "string (en|ur)",
    "learning_preferences": {
      "difficulty_level": "string",
      "learning_pace": "string (slow|moderate|fast)",
      "preferred_formats": ["string (text|video|interactive)"]
    }
  }
  ```
- **Response**:
  - 200 OK
  ```json
  {
    "message": "Preferences updated successfully"
  }
  ```

### Get personalized learning path
- **GET** `/learning-path`
- **Description**: Retrieve a personalized learning path based on user progress and preferences
- **Authentication**: Required (JWT token)
- **Response**:
  - 200 OK
  ```json
  {
    "recommended_modules": [
      {
        "id": "string",
        "title": "string",
        "reason": "string (why recommended)",
        "estimated_time": "integer (minutes)"
      }
    ],
    "next_chapters": [
      {
        "id": "string",
        "title": "string",
        "module_title": "string",
        "reason": "string (why recommended next)"
      }
    ],
    "review_suggestions": [
      {
        "id": "string",
        "title": "string",
        "module_title": "string",
        "reason": "string (why should review)"
      }
    ]
  }
  ```

### Update chapter progress
- **PUT** `/progress/chapter/{chapter_id}`
- **Description**: Update progress for a specific chapter
- **Authentication**: Required (JWT token)
- **Request Body**:
  ```json
  {
    "status": "string (not-started|in-progress|completed)",
    "completion_percentage": "integer (0-100)",
    "time_spent": "integer (seconds)"
  }
  ```
- **Response**:
  - 200 OK
  ```json
  {
    "message": "Progress updated successfully"
  }
  ```

### Get user progress summary
- **GET** `/progress`
- **Description**: Retrieve overall progress summary for the user
- **Authentication**: Required (JWT token)
- **Response**:
  - 200 OK
  ```json
  {
    "progress_summary": {
      "completed_modules": "integer",
      "total_modules": "integer",
      "completed_chapters": "integer",
      "total_chapters": "integer",
      "total_learning_time": "integer (minutes)",
      "average_completion_rate": "number (0-100)",
      "streak_days": "integer"
    },
    "progress_by_module": [
      {
        "module_id": "string",
        "module_title": "string",
        "completed_chapters": "integer",
        "total_chapters": "integer",
        "completion_percentage": "number (0-100)"
      }
    ]
  }
  ```

## Error Responses

All endpoints may return:
- 400 Bad Request: Invalid request parameters
- 401 Unauthorized: Missing or invalid authentication token
- 404 Not Found: User or resource not found
- 409 Conflict: Email already registered
- 422 Unprocessable Entity: Invalid data format
- 500 Internal Server Error: Server error

## Rate Limiting
- 100 requests per hour per user for authenticated endpoints
- 10 requests per hour per IP for unauthenticated endpoints
- 429 Too Many Requests if exceeded