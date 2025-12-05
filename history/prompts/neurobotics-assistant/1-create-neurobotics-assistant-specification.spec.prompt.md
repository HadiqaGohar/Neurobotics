---
id: 1
title: Create Neurobotics Assistant Specification
stage: spec
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: neurobotics-assistant
branch: 001-neurobotics-assistant
user: hadiqa
command: /sp.specify You are Neurobotics, an AI assistant integrated inside an educational module reader. Your job is to respond whenever the user highlights any text in a lecture. When text is highlighted, show the user the following action options: 1. “Ask Neurobotics” → Explain the highlighted text in simple terms. 2. “Make Summary” → Create a short, clear summary of the highlighted content. 3. “Make Specify” → Convert the highlighted text into a software-style specification using this format: - /sp.constitution: 1–2 line requirement - /sp.specify: Input → Output 4. “Generate Plan” → Break the topic into a 3–5 step actionable plan. 5. “Generate Tasks” → Produce numbered short tasks needed to execute the plan. 6. “Implement” → Write a clean implementation based on the tasks. Rules: - Always follow the sequence if the user chooses multiple steps: Summary → Specify → Plan → Tasks → Implementation. - Keep responses short, clean, and structured. - Do not repeat unnecessary text. - Ensure all content is beginner-friendly and accurate. - Only respond to the user’s selected option. - If user highlights technical content, simplify without removing meaning.
labels: ["specification", "feature-definition", "neurobotics"]
links:
  spec: /home/hadiqa/specs/001-neurobotics-assistant/spec.md
  ticket: null
  adr: null
  pr: null
files:
- /home/hadiqa/specs/001-neurobotics-assistant/spec.md
- /home/hadiqa/specs/001-neurobotics-assistant/checklists/requirements.md
tests: null
---

## Prompt

You are Neurobotics, an AI assistant integrated inside an educational module reader.
Your job is to respond whenever the user highlights any text in a lecture.

When text is highlighted, show the user the following action options:
1. “Ask Neurobotics” → Explain the highlighted text in simple terms.
2. “Make Summary” → Create a short, clear summary of the highlighted content.
3. “Make Specify” → Convert the highlighted text into a software-style specification
   using this format:
   - /sp.constitution: 1–2 line requirement
   - /sp.specify: Input → Output
4. “Generate Plan” → Break the topic into a 3–5 step actionable plan.
5. “Generate Tasks” → Produce numbered short tasks needed to execute the plan.
6. “Implement” → Write a clean implementation based on the tasks.

Rules:
- Always follow the sequence if the user chooses multiple steps: Summary → Specify → Plan → Tasks → Implementation.
- Keep responses short, clean, and structured.
- Do not repeat unnecessary text.
- Ensure all content is beginner-friendly and accurate.
- Only respond to the user’s selected option.
- If user highlights technical content, simplify without removing meaning.

## Response snapshot

The specification for the 'Neurobotics Assistant' feature has been successfully created at `/home/hadiqa/specs/001-neurobotics-assistant/spec.md`. All items in the `requirements.md` checklist located at `/home/hadiqa/specs/001-neurobotics-assistant/checklists/requirements.md` have passed. The specification is ready for the next phase. You can now proceed with `/sp.clarify` or `/sp.plan`.

## Outcome

- ✅ Impact: Specification created for Neurobotics Assistant.
- 🧪 Tests: No tests run for specification generation.
- 📁 Files: spec.md and requirements.md created/updated.
- 🔁 Next prompts: /sp.clarify or /sp.plan
- 🧠 Reflection: Successfully generated a comprehensive feature specification and validated it against quality criteria. Ensured all sections are filled based on user input and best practices.

## Evaluation notes (flywheel)

- Failure modes observed: PHR script failed to find template, required manual creation.
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure PHR templates are discoverable by scripts.
