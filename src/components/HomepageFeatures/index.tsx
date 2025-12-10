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
    title: 'Module 1: ROS 2 Fundamentals',
    Svg: require('@site/static/img/robot-node.svg').default,
    description: (
      <>
        Learn the middleware that coordinates humanoid robot software—nodes,
        topics, services, QoS policies, and URDF robot descriptions.
      </>
    ),
  },
  {
    title: 'Module 2: Digital Twin',
    Svg: require('@site/static/img/digital-twin.svg').default,
    description: (
      <>
        Explore physics simulation with Gazebo, photorealistic rendering
        with Unity, and sensor modeling for synthetic data generation.
      </>
    ),
  },
  {
    title: 'Module 3: AI-Robot Brain',
    Svg: require('@site/static/img/ai-brain.svg').default,
    description: (
      <>
        Master GPU-accelerated perception with NVIDIA Isaac Sim, cuVSLAM visual
        odometry, and Nav2 path planning for bipedal navigation.
      </>
    ),
  },
  {
    title: 'Module 4: Vision-Language-Action',
    Svg: require('@site/static/img/vla-system.svg').default,
    description: (
      <>
        Integrate speech recognition with Whisper, LLM task planning, and
        ROS 2 action sequencing for natural language robot control.
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--3')}>
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
