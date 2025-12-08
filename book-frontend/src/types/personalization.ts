/**
 * Types for personalization features
 */

export enum SoftwareCategory {
  WEB_DEVELOPMENT = 'web_development',
  MOBILE_DEVELOPMENT = 'mobile_development',
  DESKTOP_DEVELOPMENT = 'desktop_development',
  DEVOPS = 'devops',
  DATA_SCIENCE = 'data_science',
  MACHINE_LEARNING = 'machine_learning',
  GAME_DEVELOPMENT = 'game_development',
  EMBEDDED_SYSTEMS = 'embedded_systems',
  CYBERSECURITY = 'cybersecurity',
  CLOUD_COMPUTING = 'cloud_computing',
}

export enum HardwareCategory {
  EMBEDDED_SYSTEMS = 'embedded_systems',
  IOT_DEVICES = 'iot_devices',
  ROBOTICS = 'robotics',
  ELECTRONICS = 'electronics',
  MICROCONTROLLERS = 'microcontrollers',
  SENSORS = 'sensors',
  NETWORKING_HARDWARE = 'networking_hardware',
  AUTOMOTIVE = 'automotive',
  AEROSPACE = 'aerospace',
  INDUSTRIAL_AUTOMATION = 'industrial_automation',
}

export enum ExperienceLevel {
  BEGINNER = 'beginner',
  INTERMEDIATE = 'intermediate',
  ADVANCED = 'advanced',
  EXPERT = 'expert',
}

export interface SoftwareBackground {
  categories: SoftwareCategory[];
  experienceLevel: ExperienceLevel;
  preferredLanguages: string[];
  frameworks: string[];
}

export interface HardwareBackground {
  categories: HardwareCategory[];
  experienceLevel: ExperienceLevel;
  platforms: string[];
  components: string[];
}

export interface UserPreferences {
  softwareBackground: SoftwareBackground;
  hardwareBackground: HardwareBackground;
  contentComplexity: 'simple' | 'moderate' | 'detailed' | 'comprehensive';
  explanationDepth: 'overview' | 'standard' | 'detailed';
  exampleStyle: 'basic' | 'practical' | 'advanced';
}

// Display labels for categories
export const SOFTWARE_CATEGORY_LABELS: Record<SoftwareCategory, string> = {
  [SoftwareCategory.WEB_DEVELOPMENT]: 'Web Development',
  [SoftwareCategory.MOBILE_DEVELOPMENT]: 'Mobile Development',
  [SoftwareCategory.DESKTOP_DEVELOPMENT]: 'Desktop Development',
  [SoftwareCategory.DEVOPS]: 'DevOps & Infrastructure',
  [SoftwareCategory.DATA_SCIENCE]: 'Data Science',
  [SoftwareCategory.MACHINE_LEARNING]: 'Machine Learning & AI',
  [SoftwareCategory.GAME_DEVELOPMENT]: 'Game Development',
  [SoftwareCategory.EMBEDDED_SYSTEMS]: 'Embedded Systems',
  [SoftwareCategory.CYBERSECURITY]: 'Cybersecurity',
  [SoftwareCategory.CLOUD_COMPUTING]: 'Cloud Computing',
};

export const HARDWARE_CATEGORY_LABELS: Record<HardwareCategory, string> = {
  [HardwareCategory.EMBEDDED_SYSTEMS]: 'Embedded Systems',
  [HardwareCategory.IOT_DEVICES]: 'IoT Devices',
  [HardwareCategory.ROBOTICS]: 'Robotics',
  [HardwareCategory.ELECTRONICS]: 'Electronics',
  [HardwareCategory.MICROCONTROLLERS]: 'Microcontrollers',
  [HardwareCategory.SENSORS]: 'Sensors',
  [HardwareCategory.NETWORKING_HARDWARE]: 'Networking Hardware',
  [HardwareCategory.AUTOMOTIVE]: 'Automotive',
  [HardwareCategory.AEROSPACE]: 'Aerospace',
  [HardwareCategory.INDUSTRIAL_AUTOMATION]: 'Industrial Automation',
};

export const EXPERIENCE_LEVEL_LABELS: Record<ExperienceLevel, string> = {
  [ExperienceLevel.BEGINNER]: 'Beginner',
  [ExperienceLevel.INTERMEDIATE]: 'Intermediate',
  [ExperienceLevel.ADVANCED]: 'Advanced',
  [ExperienceLevel.EXPERT]: 'Expert',
};

// Common programming languages
export const PROGRAMMING_LANGUAGES = [
  'Python',
  'JavaScript',
  'TypeScript',
  'Java',
  'C++',
  'C',
  'C#',
  'Go',
  'Rust',
  'Swift',
  'Kotlin',
  'PHP',
  'Ruby',
  'Scala',
  'R',
  'MATLAB',
  'SQL',
  'HTML',
  'CSS',
  'Bash',
  'PowerShell',
];

// Common frameworks
export const FRAMEWORKS = [
  'React',
  'Angular',
  'Vue.js',
  'Node.js',
  'Express.js',
  'Django',
  'Flask',
  'FastAPI',
  'Spring Boot',
  'Laravel',
  'Ruby on Rails',
  'ASP.NET',
  'Flutter',
  'React Native',
  'Xamarin',
  'Unity',
  'TensorFlow',
  'PyTorch',
  'Pandas',
  'NumPy',
];

// Hardware platforms
export const HARDWARE_PLATFORMS = [
  'Arduino',
  'Raspberry Pi',
  'ESP32',
  'ESP8266',
  'STM32',
  'PIC',
  'ARM',
  'FPGA',
  'BeagleBone',
  'NVIDIA Jetson',
  'Intel NUC',
  'Teensy',
  'micro:bit',
];

// Hardware components
export const HARDWARE_COMPONENTS = [
  'Sensors',
  'Actuators',
  'Motors',
  'Displays',
  'Communication Modules',
  'Power Management',
  'Memory',
  'Processors',
  'Analog Circuits',
  'Digital Circuits',
  'PCB Design',
  'Mechanical Components',
];