import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx(styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.textContent}>
            <div className={styles.badge}>COMPREHENSIVE COURSE</div>
            <Heading as="h1" className={clsx(styles.heroTitle, 'hero__title')}>
              {siteConfig.title}
            </Heading>
            <p className={clsx(styles.heroSubtitle, 'hero__subtitle')}>
              {siteConfig.tagline}
            </p>
            <div className={styles.buttons}>
              <Link className={clsx(styles.primaryButton, 'button button--primary button--lg')} to="/docs/intro">
                Start Learning Now
              </Link>
              <Link className={clsx(styles.secondaryButton, 'button button--secondary button--lg')} to="/docs/about/course-overview">
                Course Overview
              </Link>
            </div>
          </div>
          <div className={styles.imageContent}>
            <div className={styles.bookCover}>
              <div className={styles.bookSpine}></div>
              <div className={styles.bookFace}>
                <div className={styles.bookTitle}>
                  <h2>Physical AI &amp;<br />Humanoid Robotics</h2>
                  <p>Complete 13-Week Course</p>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeaturesSection() {
  const features = [
    {
      title: "Complete Curriculum",
      description: "13 weeks of comprehensive content covering ROS 2, simulation, AI integration, and human-robot interaction.",
      icon: "📚"
    },
    {
      title: "Hands-On Learning",
      description: "Practical exercises, coding tutorials, and real-world applications to reinforce concepts.",
      icon: "🛠️"
    },
    {
      title: "Industry-Ready Skills",
      description: "Master technologies used in modern robotics including NVIDIA Isaac, Gazebo, and Unity.",
      icon: "🤖"
    }
  ];

  return (
    <section className={styles.featuresSection}>
      <div className="container">
        <div className={styles.featuresGrid}>
          {features.map((feature, index) => (
            <div key={index} className={styles.featureCard}>
              <div className={styles.featureIcon}>{feature.icon}</div>
              <h3 className={styles.featureTitle}>{feature.title}</h3>
              <p className={styles.featureDescription}>{feature.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function ModulesOverview() {
  const modules = [
    {
      title: "Module 1: Robotic Nervous System",
      weeks: "Weeks 1-5",
      description: "Introduction to Physical AI and ROS 2 fundamentals"
    },
    {
      title: "Module 2: Digital Twin",
      weeks: "Weeks 6-7",
      description: "Gazebo and Unity simulation environments"
    },
    {
      title: "Module 3: AI-Robot Brain",
      weeks: "Weeks 8-10",
      description: "NVIDIA Isaac for perception and planning"
    },
    {
      title: "Module 4: Vision-Language-Action",
      weeks: "Weeks 11-13",
      description: "Human-robot interaction and capstone project"
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" className={styles.sectionTitle}>Course Structure</Heading>
          <p className={styles.sectionSubtitle}>Four comprehensive modules covering the complete journey from digital AI to physical embodied intelligence</p>
        </div>

        <div className={styles.modulesGrid}>
          {modules.map((module, index) => (
            <div key={index} className={styles.moduleCard}>
              <div className={styles.moduleHeader}>
                <span className={styles.moduleWeeks}>{module.weeks}</span>
                <h3 className={styles.moduleTitle}>{module.title}</h3>
              </div>
              <p className={styles.moduleDescription}>{module.description}</p>
              <Link className={styles.moduleLink} to={`/docs/module-${index + 1}`}>
                Explore Module
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" className={styles.ctaTitle}>Ready to Start Your Robotics Journey?</Heading>
          <p className={styles.ctaSubtitle}>Join thousands of students learning the future of robotics</p>
          <div className={styles.ctaButtons}>
            <Link className={clsx(styles.primaryButton, 'button button--primary button--lg')} to="/docs/intro">
              Begin the Course
            </Link>
            <Link className={clsx(styles.outlineButton, 'button button--outline button--secondary button--lg')} to="/docs/about/prerequisites">
              Check Prerequisites
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home | ${siteConfig.title}`}
      description="Complete textbook for designing, simulating, and deploying humanoid robots with natural human interactions">
      <HomepageHeader />
      <main>
        <FeaturesSection />
        <ModulesOverview />
        <CTASection />
      </main>
    </Layout>
  );
}