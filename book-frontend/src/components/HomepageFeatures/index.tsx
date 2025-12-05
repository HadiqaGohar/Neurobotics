import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import { 
  FaRobot, 
  FaCogs, 
  FaCube, 
  FaEye, 
  FaHandshake 
} from 'react-icons/fa';
import styles from './styles.module.css';

type ModuleItem = {
  number: string;
  title: string;
  description: string;
  Icon: React.ComponentType<React.ComponentProps<'svg'>>;
  topics: string[];
  weeks: string;
  link: string;
  color: string;
};

const ModuleList: ModuleItem[] = [
  {
    number: '1',
    title: 'Introduction to Physical AI',
    description: 'Master the robotic nervous system with ROS 2 middleware for seamless control.',
    Icon: FaRobot,
    topics: ['Robotics Topics and Services', 'Python agents with rospy', 'URDF for Humanoids'],
    weeks: 'WEEKS 1-5',
    link: '/docs/module-1/intro',
    color: 'blue'
  },
  {
    number: '2', 
    title: 'ROS 2 - The Robot Operating System',
    description: 'Build physics simulations and high-fidelity environments.',
    Icon: FaCogs,
    topics: ['Gazebo Physics simulation', 'Unity Visualization', 'Sensor Simulation (LiDAR, IMU)'],
    weeks: 'WEEKS 6-7',
    link: '/docs/module-2/intro',
    color: 'purple'
  },
  {
    number: '3',
    title: 'Robot Simulation and Digital Twins', 
    description: 'Advanced perception, navigation, and sim-to-real transfer.',
    Icon: FaCube,
    topics: ['Isaac Sim & Synthetic Data', 'VSLAM & Navigation', 'Sim-to-real Learning'],
    weeks: 'WEEKS 8-10',
    link: '/docs/module-3/intro',
    color: 'green'
  },
  {
    number: '4',
    title: 'AI-Powered Robot Perception',
    description: 'Convergence of LLM and Robotics for conversational control.',
    Icon: FaEye,
    topics: ['Voice to Action (Whisper)', 'LLM Cognitive Planning', 'Autonomous Humanoid Captione'],
    weeks: 'WEEKS 11-13',
    link: '/docs/module-4/intro', 
    color: 'orange'
  },
  {
    number: '5',
    title: 'Human-Robot Interaction',
    description: 'Advanced multimodal AI for natural human-robot communication.',
    Icon: FaHandshake,
    topics: ['Vision-Language Models', 'Gesture Recognition', 'Social Robotics'],
    weeks: 'WEEKS 14-16',
    link: '/docs/module-5/intro',
    color: 'red'
  }
];

function Module({number, title, description, Icon, topics, weeks, link, color}: ModuleItem) {
  return (
    <div className={clsx(styles.moduleCard, styles[`module${color}`])}>
      <div className={styles.moduleHeader}>
        <div className={styles.moduleIcon}>
          <Icon />
        </div>
        <div className={styles.moduleNumber}>MODULE {number}</div>
      </div>
      
      <div className={styles.moduleContent}>
        <Heading as="h3" className={styles.moduleTitle}>{title}</Heading>
        <p className={styles.moduleDescription}>{description}</p>
        
        <ul className={styles.topicsList}>
          {topics.map((topic, idx) => (
            <li key={idx}>{topic}</li>
          ))}
        </ul>
        
        <div className={styles.moduleFooter}>
          <span className={styles.moduleWeeks}>{weeks}</span>
          <Link to={link} className={styles.moduleLink}>
            Get Started →
          </Link>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.badge}>YOUR JOURNEY</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Master Physical AI Through Hands-On Learning
          </Heading>
          <p className={styles.sectionSubtitle}>
            Build intelligent robots that understand and interact with the real world through structured modules
          </p>
        </div>
        
        <div className={styles.modulesGrid}>
          {ModuleList.map((props, idx) => (
            <Module key={idx} {...props} />
          ))}
        </div>
        

      </div>
    </section>
  );
}
