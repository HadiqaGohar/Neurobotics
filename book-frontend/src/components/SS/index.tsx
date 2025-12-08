import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import { 
  FaQuoteLeft,
  FaLinkedin,
  FaTwitter,
  FaGithub,
  FaStar,
  FaRocket
} from 'react-icons/fa';
import styles from './styles.module.css';

type StoryItem = {
  name: string;
  role: string;
  company: string;
  avatar: string;
  quote: string;
  achievement: string;
  linkedin?: string;
  twitter?: string;
  github?: string;
  rating: number;
};

const StoriesData: StoryItem[] = [
  {
    name: 'Sarah Chen',
    role: 'Robotics Engineer',
    company: 'Tesla Autopilot',
    avatar: 'SC',
    quote: 'Neurobotics AI transformed my career. From web developer to robotics engineer in 8 months. The sim-to-real transfer modules were game-changing.',
    achievement: 'Built autonomous delivery robot',
    linkedin: '#',
    github: '#',
    rating: 5
  },
  {
    name: 'Marcus Rodriguez',
    role: 'AI Research Scientist',
    company: 'Boston Dynamics',
    avatar: 'MR',
    quote: 'The VLA models and human-robot interaction modules gave me the foundation to work on next-gen humanoid robots. Incredible depth and practical focus.',
    achievement: 'Published 3 papers on embodied AI',
    twitter: '#',
    github: '#',
    rating: 5
  },
  {
    name: 'Priya Patel',
    role: 'Startup Founder',
    company: 'RoboHealth Inc.',
    avatar: 'PP',
    quote: 'Started with zero robotics knowledge. Now running a healthcare robotics startup valued at $2M. The business applications are endless.',
    achievement: 'Raised $500K seed funding',
    linkedin: '#',
    twitter: '#',
    rating: 5
  }
];

type StatItem = {
  number: string;
  label: string;
  description: string;
};

const Stats: StatItem[] = [
  {
    number: '10,000+',
    label: 'Students Trained',
    description: 'Professionals upskilled in Physical AI'
  },
  {
    number: '85%',
    label: 'Career Advancement',
    description: 'Got promotions or new roles'
  },
  {
    number: '$75K',
    label: 'Average Salary Increase',
    description: 'After completing the program'
  },
  {
    number: '200+',
    label: 'Robots Deployed',
    description: 'Built by our graduates'
  }
];

function SuccessStory({name, role, company, avatar, quote, achievement, linkedin, twitter, github, rating}: StoryItem) {
  return (
    <div className={styles.storyCard}>
      <div className={styles.storyHeader}>
        <FaQuoteLeft className={styles.quoteIcon} />
        <div className={styles.rating}>
          {[...Array(rating)].map((_, i) => (
            <FaStar key={i} className={styles.star} />
          ))}
        </div>
      </div>
      
      <blockquote className={styles.quote}>
        "{quote}"
      </blockquote>
      
      <div className={styles.storyFooter}>
        <div className={styles.authorInfo}>
          <div className={styles.avatar}>{avatar}</div>
          <div className={styles.authorDetails}>
            <h4 className={styles.authorName}>{name}</h4>
            <p className={styles.authorRole}>{role} at {company}</p>
            <p className={styles.achievement}>🏆 {achievement}</p>
          </div>
        </div>
        
        <div className={styles.socialLinks}>
          {linkedin && <a href={linkedin}><FaLinkedin /></a>}
          {twitter && <a href={twitter}><FaTwitter /></a>}
          {github && <a href={github}><FaGithub /></a>}
        </div>
      </div>
    </div>
  );
}

function Stat({number, label, description}: StatItem) {
  return (
    <div className={styles.statItem}>
      <div className={styles.statNumber}>{number}</div>
      <div className={styles.statLabel}>{label}</div>
      <div className={styles.statDescription}>{description}</div>
    </div>
  );
}

export default function SuccessStories(): ReactNode {
  return (
    <section className={styles.successSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <span className={styles.badge}>READY TO BEGIN?</span>
          <Heading as="h2" className={styles.sectionTitle}>
            The Future is Physical AI
            <br />
            <span className={styles.highlight}>Robots That Think, Move, and Collaborate</span>
          </Heading>
          <p className={styles.sectionSubtitle}>
            Join the transition from AI confined to screens to AI that shapes the physical world alongside us.
          </p>
        </div>
        
        <div className={styles.statsGrid}>
          {Stats.map((stat, idx) => (
            <Stat key={idx} {...stat} />
          ))}
        </div>
        
        <div className={styles.storiesGrid}>
          {StoriesData.map((story, idx) => (
            <SuccessStory key={idx} {...story} />
          ))}
        </div>
        
        <div className={styles.ctaSection}>
          <div className={styles.ctaContent}>
            <FaRocket className={styles.ctaIcon} />
            <div className={styles.ctaText}>
              <h3>Start Your Physical AI Journey</h3>
              <p>From ROS 2 basics to autonomous humanoids with voice control</p>
            </div>
            <Link to="/docs/intro" className={styles.ctaButton}>
              Get Started Free
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}