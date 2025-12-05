import type { SidebarsConfig } from "@docusaurus/plugin-content-docs";

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    "intro",
    "book-overview",
    {
      type: "category",
      label: "Module 1: Introduction to Physical AI",
      items: [
        "module-1/lecture-1",
        "module-1/lecture-2",
        "module-1/lecture-3",
        "module-1/lecture-4",
        "module-1/lecture-5",
      ],
    },
    {
      type: "category",
      label: "Module 2: ROS 2 - The Robot Operating System",
      items: [
        "module-2/lecture-1",
        "module-2/lecture-2",
        "module-2/lecture-3",
        "module-2/lecture-4",
        "module-2/lecture-5",
      ],
    },
    {
      type: "category",
      label: "Module 3: Robot Simulation and Digital Twins",
      items: [
        "module-3/lecture-1",
        "module-3/lecture-2",
        "module-3/lecture-3",
        "module-3/lecture-4",
        "module-3/lecture-5",
      ],
    },
    {
      type: "category",
      label: "Module 4: AI-Powered Robot Perception",
      items: [
        "module-4/lecture-1",
        "module-4/lecture-2",
        "module-4/lecture-3",
        "module-4/lecture-4",
        "module-4/lecture-5",
      ],
    },
    {
      type: "category",
      label: "Module 5: Human-Robot Interaction",
      items: [
        "module-5/lecture-1",
        "module-5/lecture-2",
        "module-5/lecture-3",
        "module-5/lecture-4",
        "module-5/lecture-5",
      ],
    },
  ],
};

export default sidebars;
