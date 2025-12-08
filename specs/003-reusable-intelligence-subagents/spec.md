# Reusable Intelligence via Claude Code Subagents & Skills

## Overview

Leverage existing Claude Code Subagents and create Agent Skills to provide reusable intelligence that enhances automation and efficiency within the book development workflow.

## Scope

### In Scope
- Utilize existing Claude subagents in `.claude/agents/` directory
- Create reusable skills for common book development tasks
- Simple skill management and organization system
- Basic utilities to list and execute available agents and skills

### Out of Scope
- Modification of existing Claude subagents
- Complex orchestration or AI routing systems
- Real-time monitoring or analytics
- External integrations beyond existing project structure

## Key Components

1. **Skills Directory**: Organized collection of reusable automation scripts
2. **Skill Manager**: Simple utility to organize and execute skills
3. **Agent Utility**: Helper to list and reference existing Claude subagents
4. **Book Development Skills**: Specific skills for content, documentation, and workflow tasks

## Success Criteria

- Skills directory created and organized
- 3-5 practical skills implemented for book development
- Simple way to list available Claude subagents
- Skills can be easily executed and reused
- Clear documentation for skill usage

## Dependencies

- Existing Claude subagents in `.claude/agents/` directory
- Basic file system access for skill storage and execution

## Acceptance Criteria

- `.claude/skills/` directory structure created
- Skill manager utility implemented
- Agent listing utility created
- At least 3 book development skills implemented and tested
- Simple documentation for using skills and agents