
import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A Comprehensive Guide to Building Intelligent Robots with ROS 2, Gazebo, and Vision-Language-Action Models',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://soobiarao14.github.io', // ✅ UPDATED: Your GitHub Pages URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/my-book-hackathon/', // ✅ UPDATED: Your repository name

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'soobiarao14', // ✅ UPDATED: Your GitHub username
  projectName: 'my-book-hackathon', // ✅ UPDATED: Your repository name

  onBrokenLinks: 'warn', // Allow build with warnings for chapters not yet written
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Disable edit links or update with your repo URL
          editUrl: 'https://github.com/soobiarao14/my-book-hackathon/tree/main/', // ✅ UPDATED & UNCOMMENTED
        },
        blog: false, // Blog disabled - not needed for book
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/social-card.jpg', // TODO: Create a social media preview image
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book Chapters',
        },
        {
          href: 'https://github.com/soobiarao14/my-book-hackathon', // ✅ UPDATED
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book Sections',
          items: [
            {
              label: 'Getting Started',
              to: '/docs',
            },
            {
              label: 'Module 1: ROS 2 Foundations',
              to: '/docs/module-1-ros2',
            },
            {
              label: 'Module 2: Digital Twin & Simulation',
              to: '/docs/module-2-digital-twin',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Hardware Requirements',
              to: '/docs/appendices/appendix-a-hardware',
            },
            {
              label: 'GitHub Repository',
              href: 'https://github.com/soobiarao14/my-book-hackathon', // ✅ UPDATED
            },
          ],
        },
        {
          title: 'Topics',
          items: [
            {
              label: 'ROS 2',
              to: '/docs/module-1-ros2',
            },
            {
              label: 'Vision-Language-Action',
              to: '/docs/module-4-vla',
            },
            {
              label: 'NVIDIA Isaac',
              to: '/docs/module-3-isaac',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;