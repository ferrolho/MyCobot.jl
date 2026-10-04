// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import starlightLinksValidator from 'starlight-links-validator';

// GitHub repository name. It sets the base path of the published site
// (https://ferrolho.github.io/<REPO>/). If the repository is renamed, change it here and replace
// `/mycobot-280-lab/` in the internal links under src/content/docs/ (the build reports any link you miss).
const REPO = 'mycobot-280-lab';

export default defineConfig({
  site: 'https://ferrolho.github.io',
  base: `/${REPO}`,
  trailingSlash: 'always',
  integrations: [
    starlight({
      title: 'myCobot 280 Lab',
      description:
        'High-rate control of the myCobot 280 (for Arduino): the servo bus, the ATOM firmware, the Julia and Python tools, and the measured results.',
      social: [{ icon: 'github', label: 'GitHub', href: `https://github.com/ferrolho/${REPO}` }],
      editLink: { baseUrl: `https://github.com/ferrolho/${REPO}/edit/main/website/` },
      customCss: ['./src/styles/custom.css'],
      // The build fails on broken internal links and anchors: the site is the source of truth.
      plugins: [starlightLinksValidator()],
      lastUpdated: true,
      sidebar: [
        {
          label: 'Start here',
          items: [
            { label: 'Overview', slug: 'start/overview' },
            { label: 'Quick start', slug: 'start/quick-start' },
            { label: 'Safety', slug: 'start/safety' },
          ],
        },
        {
          label: 'System',
          items: [
            { label: 'Architecture', slug: 'system/architecture' },
            { label: 'Robot and wiring', slug: 'system/robot' },
            { label: 'Servos', slug: 'system/servos' },
            { label: 'ATOM controller board', slug: 'system/atom' },
            { label: 'Gripper', slug: 'system/gripper' },
          ],
        },
        {
          label: 'Communication',
          items: [
            { label: 'Servo bus protocol', slug: 'comms/servo-bus' },
            { label: 'Laptop link (FT232)', slug: 'comms/laptop-link' },
            { label: 'ATOM link (WiFi)', slug: 'comms/atom-link' },
            { label: 'Stock ATOM protocol', slug: 'comms/stock-atom' },
          ],
        },
        {
          label: 'Firmware',
          items: [
            { label: 'Controller firmware', slug: 'firmware/controller' },
            { label: 'LED matrix signals', slug: 'firmware/led-signals' },
            { label: 'Bus probe firmware', slug: 'firmware/probe' },
            { label: 'Build, flash and update', slug: 'firmware/build-flash' },
            { label: 'Stock firmware backup', slug: 'firmware/stock-backup' },
          ],
        },
        {
          label: 'Software',
          items: [
            { label: 'Julia package', slug: 'software/julia' },
            { label: 'Scripts', slug: 'software/scripts' },
            { label: 'Python tools', slug: 'software/python' },
            { label: 'Raspberry Pi 5', slug: 'software/raspberry-pi' },
            { label: 'Trajectory optimisation (TORA)', slug: 'software/tora' },
            { label: 'Tests', slug: 'software/tests' },
          ],
        },
        {
          label: 'Results',
          items: [
            { label: 'Latency and loop rate', slug: 'results/latency' },
            { label: 'Servo response', slug: 'results/servo-response' },
            { label: 'Circle and lag compensation', slug: 'results/circle' },
            { label: 'Iterative learning control', slug: 'results/ilc' },
            { label: 'Onboard control and vibration', slug: 'results/onboard' },
          ],
        },
        {
          label: 'Reference',
          items: [
            { label: 'Known problems and rules', slug: 'reference/gotchas' },
            { label: 'Servo register map', slug: 'reference/registers' },
            { label: 'Stock protocol commands', slug: 'reference/stock-protocol' },
            { label: 'Roadmap', slug: 'reference/roadmap' },
            { label: 'History', slug: 'reference/history' },
            { label: 'Writing style', slug: 'reference/writing-style' },
          ],
        },
      ],
    }),
  ],
});
