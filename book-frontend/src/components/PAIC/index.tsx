import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import { 
  FaBrain, 
  FaUsers, 
  FaIndustry, 
  FaComments, 
  FaGlobe, 
  FaRocket 
} from 'react-icons/fa';
import styles from './styles.module.css';

type CapabilityItem = {
  title: string;
  description: string;
  Icon: React.ComponentType<React.ComponentProps<'svg'>>;
  badge?: string;
  color: string;
};

const CapabilityList: CapabilityItem[] = [
  {
    title: 'Embodied Intelligence',
    description: 'AI that operates in physical space, not just digital environments. Robots that understand physics and interact with the real world.',
    Icon: FaBrain,
    badge: 'CORE',
    color: 'blue'
  },
  {
    title: 'Human-Centered Design',
    description: 'Humanoid robots navigate our world without modification. They use human tools, interfaces, and learn from demonstrations.',
    Icon: FaUsers,
    color: 'purple'
  },
  {
    title: 'Production-Ready Skills',
    description: 'ROS 2 Gazebo, NVIDIA Isaac, and VLA models. The complete stack for modern robotics development.',
    Icon: FaIndustry,
    color: 'green'
  },
  {
    title: 'Conversational Robotics',
    description: 'Natural language commands translated to robot actions. Clean "Hi robot" becomes a sequence of coordinated movements.',
    Icon: FaComments,
    badge: 'CORE',
    color: 'orange'
  },
  {
    title: 'Sim-to-Real Transfer',
    description: 'Train in simulation, deploy to reality. Photorealistic environments and domain randomization bridge the gap.',
    Icon: FaGlobe,
    color: 'teal'
  },
  {
    title: 'Interactive Learning',
    description: 'RAG-powered chat, personalized content and exercises. Learn by doing, not just reading.',
    Icon: FaRocket,
    color: 'red'
  }
];

function Capability({title, description, Icon, badge, color}: CapabilityItem) {
  return (
    <div className={clsx(styles.capabilityCard, styles[`capability${color}`])}>
      {badge && (
        <div className={styles.capabilityBadge}>{badge}</div>
      )}
      
      <div className={styles.capabilityIcon}>
        <Icon />
      </div>
      
      <div className={styles.capabilityContent}>
        <Heading as="h3" className={styles.capabilityTitle}>{title}</Heading>
        <p className={styles.capabilityDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function PhysicalAICapabilities(): ReactNode {
  return (
    <section className={styles.capabilities}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.badge}>WHY THIS MATTERS</span>
          <Heading as="h2" className={styles.sectionTitle}>
            Build Machines That Free Up Your Time
          </Heading>
          <p className={styles.sectionSubtitle}>
            Robots that handle physical tasks while you focus on what matters most
          </p>
        </div>
        
        <div className={styles.capabilitiesGrid}>
          {CapabilityList.map((props, idx) => (
            <Capability key={idx} {...props} />
          ))}
        </div>
        
        <div className={styles.ctaSection}>
          <Link to="/docs/intro" className={styles.ctaButton}>
            Start Your Journey
          </Link>
        </div>
      </div>
    </section>
  );
}