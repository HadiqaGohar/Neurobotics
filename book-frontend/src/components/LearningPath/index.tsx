import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import { 
  FaDesktop, 
  FaCloud, 
  FaRobot,
  FaGraduationCap
} from 'react-icons/fa';
import styles from './styles.module.css';

type PathItem = {
  number: string;
  title: string;
  subtitle: string;
  description: string;
  features: string[];
  cost: string;
  badge: string;
  badgeColor: string;
  Icon: React.ComponentType<React.ComponentProps<'svg'>>;
  color: string;
};

const PathList: PathItem[] = [
  {
    number: '1',
    title: 'Browser-Based Learning',
    subtitle: 'Full Local Experience',
    description: 'Start with web-based simulations and ROS 2 tutorials. Run basic robotics code locally with our interactive environment.',
    features: [
      'Interactive ROS 2 tutorials',
      'Python robotics exercises', 
      'URDF model visualization',
      'Local development setup'
    ],
    cost: 'Free - no hardware',
    badge: 'BEST EXPERIENCE',
    badgeColor: 'blue',
    Icon: FaDesktop,
    color: 'blue'
  },
  {
    number: '2',
    title: 'Cloud + Simulation',
    subtitle: 'Hybrid Approach',
    description: 'AWS/Azure GPU instances for simulation. Gazebo kit for physics deployment with Isaac Sim integration.',
    features: [
      'Gazebo Physics simulation',
      'Unity Visualization',
      'Isaac Sim & Synthetic Data',
      'Cloud GPU acceleration'
    ],
    cost: '$20/month cloud + free Gazebo',
    badge: 'RECOMMENDED',
    badgeColor: 'orange',
    Icon: FaCloud,
    color: 'purple'
  },
  {
    number: '3',
    title: 'Physical Robot Lab',
    subtitle: 'Complete Robot Setup',
    description: 'Cloud-based simulation without physical hardware. Complete the theory and simulation modules with real robot integration.',
    features: [
      'Real humanoid robot access',
      'Physical sensor integration',
      'Sim-to-real transfer',
      'Production deployment'
    ],
    cost: 'Robot hardware + sensors',
    badge: 'ADVANCED',
    badgeColor: 'green',
    Icon: FaRobot,
    color: 'green'
  }
];

function PathOption({number, title, subtitle, description, features, cost, badge, badgeColor, Icon, color}: PathItem) {
  return (
    <div className={clsx(styles.pathCard, styles[`path${color}`])}>
      <div className={styles.pathHeader}>
        <div className={styles.pathNumber}>{number}</div>
        <div className={clsx(styles.pathBadge, styles[`badge${badgeColor}`])}>
          {badge}
        </div>
      </div>
      
      <div className={styles.pathIcon}>
        <Icon />
      </div>
      
      <div className={styles.pathContent}>
        <Heading as="h3" className={styles.pathTitle}>{title}</Heading>
        <p className={styles.pathSubtitle}>{subtitle}</p>
        <p className={styles.pathDescription}>{description}</p>
        
        <ul className={styles.featuresList}>
          {features.map((feature, idx) => (
            <li key={idx}>{feature}</li>
          ))}
        </ul>
        
        <div className={styles.pathFooter}>
          <span className={styles.pathCost}>Cost: {cost}</span>
        </div>
      </div>
    </div>
  );
}

export default function LearningPath(): ReactNode {
  return (
    <section className={styles.learningPath}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.badge}>START WHERE YOU ARE</span>
          <Heading as="h2" className={styles.sectionTitle}>
            No Expensive Hardware Required
          </Heading>
          <p className={styles.sectionSubtitle}>
            Begin building today with just your browser. Scale up when you're ready.
          </p>
        </div>
        
        <div className={styles.pathGrid}>
          {PathList.map((props, idx) => (
            <PathOption key={idx} {...props} />
          ))}
        </div>
        
        <div className={styles.ctaSection}>
          <div className={styles.ctaContent}>
            <FaGraduationCap className={styles.ctaIcon} />
            <div className={styles.ctaText}>
              <h3>Ready to Start Your Physical AI Journey?</h3>
              <p>Choose your learning path and begin building intelligent robots today</p>
            </div>
            <Link to="/docs/intro" className={styles.ctaButton}>
              Start Learning
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}