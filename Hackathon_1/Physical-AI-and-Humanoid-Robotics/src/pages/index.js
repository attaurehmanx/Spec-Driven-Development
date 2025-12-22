import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              Master the future of robotics with cutting-edge curriculum combining AI,
              humanoid robotics, and practical applications. Join thousands of learners
              advancing their robotics expertise.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Start Reading →
              </Link>
            </div>
          </div>
          <div className={styles.heroImage}>
            <img
              src={require('@site/static/img/Best-Robot-Wallpaper-1-removebg-preview.png').default}
              alt="Robotics and AI Education"
              className={styles.heroRobotImage}
            />
          </div>
        </div>
      </div>
    </header>
  );
}

function StatsSection() {
  return (
    <section className={styles.statsSection}>
      <div className="container">
        <div className={styles.statsGrid}>
          <div className={styles.statItem}>
            <div className={styles.statNumber}>4</div>
            <div className={styles.statLabel}>Modules</div>
          </div>
          <div className={styles.statItem}>
            <div className={styles.statNumber}>100%</div>
            <div className={styles.statLabel}>Open Source</div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI and Humanoid Robotics Education Platform - Learn advanced robotics, ROS 2, and AI integration">
      <HomepageHeader />
      <StatsSection />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
