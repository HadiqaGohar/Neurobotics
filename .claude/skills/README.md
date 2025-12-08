# Claude Skills Directory

## Overview
This directory contains reusable Claude Skills for book development workflows. Each skill is designed to automate common tasks and provide consistent, reliable functionality.

## Directory Structure
```
.claude/skills/
├── format-content/      # Content formatting skill
│   └── SKILL.md
├── update-readme/       # README documentation skill
│   └── SKILL.md
├── project-status/      # Project status reporting skill
│   └── SKILL.md
├── agent-discovery/     # Agent discovery utility skill
│   └── SKILL.md
├── skill-manager/       # Skill management meta-skill
│   └── SKILL.md
└── README.md           # This file
```

## Available Skills

### Content Skills
- **format-content**: Format and validate markdown content with consistent styling

### Documentation Skills  
- **update-readme**: Generate and maintain README documentation

### Workflow Skills
- **project-status**: Check project status and generate reports

### Utility Skills
- **agent-discovery**: Discover and list available Claude subagents
- **skill-manager**: Manage and organize all available skills

## Usage
Skills are automatically invoked by Claude based on context and conversation triggers. Each skill has specific context triggers that activate it when relevant topics are discussed.

## Integration with Subagents
These skills work alongside existing Claude subagents in the `.claude/agents/` directory. Use the agent-discovery skill to see what subagents are available and how they complement these skills.

## Adding New Skills
To add a new skill:
1. Create a new directory in `.claude/skills/`
2. Add a `SKILL.md` file with YAML frontmatter containing `name` and `description`
3. Include context triggers and capabilities
4. Add any supporting files as needed
5. Update this README to include the new skill

## Official Claude Skills Format
All skills follow the official Claude Skills format:
- Each skill is a directory containing `SKILL.md`
- YAML frontmatter with `name` and `description` fields
- Model-invoked based on context (not user-invoked)
- Optional supporting files (scripts, templates, docs)
- Progressive disclosure for efficient context management