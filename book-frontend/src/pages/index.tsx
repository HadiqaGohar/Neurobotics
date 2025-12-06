import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import PhysicalAICapabilities from '@site/src/components/PhysicalAICapabilities';
import LearningPath from '@site/src/components/LearningPath';
import SuccessStories from '@site/src/components/SuccessStories';
import ChatBotWrapper from '@site/src/components/ChatBot/ChatBotWrapper'; // Import ChatBotWrapper
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx(styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroLeft}>
            <div className={styles.brandBadge}>
              <span className={styles.brandName}>NEUROBOTICS AI</span>
              <span className={styles.brandTag}>PLATFORM</span>
            </div>

            <Heading as="h1" className={styles.heroTitle}>
              Build AI systems that
              <br />
              <span className={styles.highlight}>understand</span>
              <br />
              the digital world.
            </Heading>

            <p className={styles.heroSubtitle}>
              Master Artificial Intelligence from browser to production. iOS &
              Isaac Sim, and Vision-Language-Action models. <span className={styles.freeTag}>Free</span> forever.
            </p>

            <div className={styles.buttons}>
              <Link
                className={clsx('button', styles.primaryButton)}
                to="/docs/intro">
                Get Started →
              </Link>
            </div>

            <div className={styles.techStack}>
              <span>• ROS 2</span>
              <span>• Isaac Sim</span>
              <span>• Robotics</span>
              <span>• VLA Models</span>
            </div>
          </div>

          <div className={styles.heroRight}>
            <div className={styles.robotInterface}>
              <div className={styles.interfaceHeader}>
                <div className={styles.dots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
                <span className={styles.interfaceTitle}>Neurobotics AI Portal</span>
              </div>

              <div className={styles.interfaceBody}>
                <div className={styles.gridLayout}>
                  <div className={styles.gridItem}>VISION</div>
                  <div className={styles.gridItem}>CAMERA</div>
                  <div className={styles.gridItem + ' ' + styles.active}>AI</div>
                  <div className={styles.gridItem}>SENSOR</div>
                </div>

                <div className={styles.statusBar}>
                  <span>• AI Assistant Running</span>
                  <span>• VLA Models</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Neurobotics AI - ${siteConfig.title}`}
      description="Build AI systems that understand the digital world. Interactive AI assistant and VLA models."
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <PhysicalAICapabilities />
        <LearningPath />
        <SuccessStories />
      </main>
      <ChatBotWrapper />
    </Layout>
  );
}