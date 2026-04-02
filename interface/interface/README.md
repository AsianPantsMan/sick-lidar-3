# React + Vite

This template provides a minimal setup to get React working in Vite with HMR and some ESLint rules.

Currently, two official plugins are available:

- [@vitejs/plugin-react](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react) uses [Babel](https://babeljs.io/) (or [oxc](https://oxc.rs) when used in [rolldown-vite](https://vite.dev/guide/rolldown)) for Fast Refresh
- [@vitejs/plugin-react-swc](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react-swc) uses [SWC](https://swc.rs/) for Fast Refresh

## React Compiler

The React Compiler is not enabled on this template because of its impact on dev & build performances. To add it, see [this documentation](https://react.dev/learn/react-compiler/installation).

## Expanding the ESLint configuration

If you are developing a production application, we recommend using TypeScript with type-aware lint rules enabled. Check out the [TS template](https://github.com/vitejs/vite/tree/main/packages/create-vite/template-react-ts) for information on how to integrate TypeScript and [`typescript-eslint`](https://typescript-eslint.io) in your project.

## Deploying With Render

This app reads the backend URL from `VITE_API_URL` at build time.

### Backend service (Render Web Service)

- Use `npm install` as the install command.
- Use `npm start` as the start command.
- Root directory: `interface/interface/backend`
- Optional environment variable: `CORS_ORIGINS=https://your-frontend-name.onrender.com`

The server automatically uses Render's `PORT` value.

### Frontend service (Render Static Site or Web Service)

- Root directory: `interface/interface`
- Build command: `npm install && npm run build`
- Publish directory: `dist` (for Static Site)
- Set `VITE_API_URL` to your backend URL, for example:
	- `https://your-backend-name.onrender.com`

Important: if `VITE_API_URL` changes, trigger a new frontend deploy so the new value is compiled into the bundle.
