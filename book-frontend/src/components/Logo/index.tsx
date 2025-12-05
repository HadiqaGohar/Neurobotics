import React from 'react';
import { FaBrain, FaRobot } from 'react-icons/fa';
import styles from './styles.module.css';

interface LogoProps {
  size?: 'small' | 'medium' | 'large';
  showText?: boolean;
}

export default function Logo({ size = 'medium', showText = true }: LogoProps) {
  return (
    <div className={`${styles.logo} ${styles[size]}`}>
      <div className={styles.iconContainer}>
        <FaBrain className={styles.brainIcon} />
        <FaRobot className={styles.robotIcon} />
      </div>
      {showText && (
        <span className={styles.logoText}>
          <span className={styles.neuro}>Neuro</span>
          <span className={styles.botics}>botics</span>
          <span className={styles.ai}>AI</span>
        </span>
      )}
    </div>
  );
}