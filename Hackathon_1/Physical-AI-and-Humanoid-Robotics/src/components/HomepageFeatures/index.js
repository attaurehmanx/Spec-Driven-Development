import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: '🤖 Advanced Robotics Curriculum',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Comprehensive modules covering ROS 2, humanoid robotics, and AI integration
        with hands-on practical exercises and real-world applications.
      </>
    ),
  },
  {
    title: '⚡ Practical Learning Approach',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Learn by doing with step-by-step tutorials, code examples, and projects
        that build your robotics expertise from fundamentals to advanced concepts.
      </>
    ),
  },
  {
    title: '🚀 Cutting-Edge Technologies',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Master modern robotics frameworks, AI algorithms, and physical computing
        with the latest tools and methodologies in robotics development.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className={styles.featureCard}>
        <div className="text--center">
          <Svg className={styles.featureSvg} role="img" />
        </div>
        <div className="text--center padding-horiz--md">
          <Heading as="h3" className={styles.featureTitle}>{title}</Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className={styles.sectionHeader}>
            <Heading as="h2" className={styles.sectionTitle}>Why Choose Our Platform</Heading>
            <p className={styles.sectionSubtitle}>Designed for the next generation of robotics engineers</p>
          </div>
        </div>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
