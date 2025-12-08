import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/__docusaurus/debug',
    component: ComponentCreator('/__docusaurus/debug', '5ff'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/config',
    component: ComponentCreator('/__docusaurus/debug/config', '5ba'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/content',
    component: ComponentCreator('/__docusaurus/debug/content', 'a2b'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/globalData',
    component: ComponentCreator('/__docusaurus/debug/globalData', 'c3c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/metadata',
    component: ComponentCreator('/__docusaurus/debug/metadata', '156'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/registry',
    component: ComponentCreator('/__docusaurus/debug/registry', '88c'),
    exact: true
  },
  {
    path: '/__docusaurus/debug/routes',
    component: ComponentCreator('/__docusaurus/debug/routes', '000'),
    exact: true
  },
  {
    path: '/about',
    component: ComponentCreator('/about', 'c49'),
    exact: true
  },
  {
    path: '/auth/callback',
    component: ComponentCreator('/auth/callback', '9a6'),
    exact: true
  },
  {
    path: '/contact',
    component: ComponentCreator('/contact', 'abe'),
    exact: true
  },
  {
    path: '/login',
    component: ComponentCreator('/login', 'a8c'),
    exact: true
  },
  {
    path: '/privacy',
    component: ComponentCreator('/privacy', 'dcf'),
    exact: true
  },
  {
    path: '/signup',
    component: ComponentCreator('/signup', 'e02'),
    exact: true
  },
  {
    path: '/terms',
    component: ComponentCreator('/terms', '49c'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '/',
    component: ComponentCreator('/', 'de9'),
    routes: [
      {
        path: '/',
        component: ComponentCreator('/', 'cd6'),
        routes: [
          {
            path: '/',
            component: ComponentCreator('/', '4c9'),
            routes: [
              {
                path: '/intro',
                component: ComponentCreator('/intro', '9fa'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1-ros2/',
                component: ComponentCreator('/module-1-ros2/', 'a0f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1-ros2/chapter-01-basics',
                component: ComponentCreator('/module-1-ros2/chapter-01-basics', '4df'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1-ros2/chapter-02-python',
                component: ComponentCreator('/module-1-ros2/chapter-02-python', 'f3b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-1-ros2/chapter-03-urdf',
                component: ComponentCreator('/module-1-ros2/chapter-03-urdf', 'b32'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2-gazebo-unity/',
                component: ComponentCreator('/module-2-gazebo-unity/', 'e2d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2-gazebo-unity/chapter-01-gazebo',
                component: ComponentCreator('/module-2-gazebo-unity/chapter-01-gazebo', 'e9c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2-gazebo-unity/chapter-02-unity',
                component: ComponentCreator('/module-2-gazebo-unity/chapter-02-unity', '198'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-2-gazebo-unity/chapter-03-sensors',
                component: ComponentCreator('/module-2-gazebo-unity/chapter-03-sensors', '665'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3-isaac-sim/',
                component: ComponentCreator('/module-3-isaac-sim/', '27f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3-isaac-sim/chapter-01-getting-started',
                component: ComponentCreator('/module-3-isaac-sim/chapter-01-getting-started', '181'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3-isaac-sim/chapter-02-ros2-integration',
                component: ComponentCreator('/module-3-isaac-sim/chapter-02-ros2-integration', '4e4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-3-isaac-sim/chapter-03-synthetic-data',
                component: ComponentCreator('/module-3-isaac-sim/chapter-03-synthetic-data', '09a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4-vla/',
                component: ComponentCreator('/module-4-vla/', 'f30'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4-vla/chapter-01-introduction',
                component: ComponentCreator('/module-4-vla/chapter-01-introduction', '0fb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4-vla/chapter-02-using-pretrained',
                component: ComponentCreator('/module-4-vla/chapter-02-using-pretrained', 'f44'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/module-4-vla/chapter-03-finetuning',
                component: ComponentCreator('/module-4-vla/chapter-03-finetuning', 'd20'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
