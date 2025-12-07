import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Foundations',
    Svg: require('@site/static/img/logo.svg').default,
    description: (
      <>
        Master Robot Operating System 2 (ROS 2) fundamentals including nodes, topics,
        services, and actions. Build real-world robotic applications from scratch.
      </>
    ),
  },
  {
    title: 'Digital Twin & Simulation',
    Svg: require('@site/static/img/logo.svg').default,
    description: (
      <>
        Create realistic robot simulations using Gazebo and Unity. Test algorithms
        in virtual environments before deploying to physical hardware.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action AI',
    Svg: require('@site/static/img/logo.svg').default,
    description: (
      <>
        Integrate cutting-edge AI models for perception, natural language understanding,
        and autonomous decision-making in humanoid robots.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
