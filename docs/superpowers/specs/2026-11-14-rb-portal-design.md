# RB project portal

Approved direction: replace the long report with a project portal, real interactive demos, experiment comparisons, and an expandable library. User authorized implementation with monitoring and iterative comparison.

## Scope
Five public destinations: overview, 3D explorer, technology, experiments, library. Separate administrator page supports login, document upload, metadata editing, draft/publish/unpublish. Vietnamese copy. Responsive, keyboard accessible, clear loading/error/empty states.

Real reference mesh and camera poses from room-floor-preserved-20261112. Playback is recorded camera trajectory over a completed static mesh, not online reconstruction or robot body pose. Preserve metric scope and provenance. No sensor inference or robot control in the web application.

## Architecture
Flask serves HTML, JSON APIs and allowlisted assets. SQLite persists document metadata; uploads live outside the static directory. Markdown is sanitized. Administrator password is provisioned through a CLI and hashed; signed cookie session, CSRF protection, request size limit and bounded login attempts. Three.js handles mesh viewing with orbit controls, layer toggles and recorded playback. Static files and curated documents are bundled so publication does not expose the repository.

## Delivery
Keep the original report as report-legacy.html. All portal code lives in its existing report directory. Install dependencies in a local venv; npm supplies Three.js locally. Include run/setup instructions, tests, and a QA record. Bind to loopback by default. Public hosting/HTTPS is a separate deployment action.
