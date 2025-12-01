import type { Config } from 'tailwindcss'

const config: Config = {
  content: [
    './pages/**/*.{js,ts,jsx,tsx,mdx}',
    './components/**/*.{js,ts,jsx,tsx,mdx}',
    './app/**/*.{js,ts,jsx,tsx,mdx}',
  ],
  theme: {
    extend: {
      colors: {
        'padel-blue': '#3B82F6',
        'padel-red': '#EF4444',
        'padel-yellow': '#FBBF24',
        'padel-green': '#10B981',
      },
    },
  },
  plugins: [],
}
export default config

