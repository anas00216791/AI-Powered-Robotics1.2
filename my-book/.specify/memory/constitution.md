<!--
Sync Impact Report:
Version: 1.0.0 (Initial ratification)
Modified Principles:
  - All principles defined (initial creation)
Added Sections:
  - Core Principles (7 principles)
  - Citation Standards
  - Technical Reproducibility
  - Deliverables & Scope
  - Governance
Templates Requiring Updates:
  ✅ plan-template.md - verified alignment with constitution check
  ✅ spec-template.md - verified alignment with success criteria
  ✅ tasks-template.md - verified alignment with testable tasks
Follow-up TODOs: None
-->

# AI & Robotics Book with RAG Chatbot Constitution

## Core Principles

### I. Accuracy via Primary Sources

All technical content MUST be derived from authoritative primary sources. Claims, methods, and implementations MUST be traceable to official documentation, peer-reviewed publications, or verified reference implementations.

**Non-negotiable rules:**
- ROS 2, Gazebo, NVIDIA Isaac, and OpenAI documentation serve as ground truth for their respective domains
- No secondary sources (blogs, tutorials, forums) may be cited without corroboration from primary sources
- Every technical assertion requires an explicit citation
- Ambiguous or conflicting sources MUST be flagged for human review

**Rationale:** Accuracy is the foundation of educational and technical content. Primary sources ensure correctness, minimize propagation of errors, and maintain the book's credibility in academic and professional contexts.

### II. Clarity for Technical Audiences

Content MUST be clear, precise, and accessible to readers with technical or academic backgrounds. Language clarity is measured and enforced.

**Non-negotiable rules:**
- Flesch-Kincaid readability grade level MUST be between 10 and 12
- Technical jargon is permitted when standard in the domain (e.g., "ROS 2 node", "action server", "inference engine")
- Each new concept MUST be defined before use
- Complex explanations MUST include concrete examples or diagrams

**Rationale:** The audience consists of engineering students, researchers, and practitioners. Content must balance technical rigor with comprehensibility, avoiding both oversimplification and unnecessary obfuscation.

### III. Reproducibility of Code, Simulations, and Experiments

All code, simulations, and experiments MUST be reproducible by readers with access to the specified tools and environments.

**Non-negotiable rules:**
- Every code snippet MUST include environment specification (OS, versions, dependencies)
- Simulations MUST specify world files, robot models, and parameter configurations
- Instructions MUST be testable: if a reader follows the steps, they MUST achieve the documented result
- Known limitations or environment-specific failures MUST be documented explicitly

**Rationale:** Reproducibility is essential for educational value and scientific integrity. Readers must be able to validate claims and build upon the work.

### IV. Rigor via Peer-Reviewed and Official Sources

Source quality directly impacts content reliability. The constitution mandates a minimum standard for source rigor.

**Non-negotiable rules:**
- At least 50% of all citations MUST be peer-reviewed publications or official documentation
- Industry white papers, technical blogs, and community tutorials are supplementary only
- Unverified claims MUST be marked as "experimental" or "unverified"
- Conflicting sources MUST be acknowledged with rationale for the chosen interpretation

**Rationale:** Peer review and official documentation provide a quality threshold. This principle prevents degradation into opinion-based or anecdotal content.

### V. Zero Plagiarism and Proper Attribution

All content MUST be original or properly attributed. Plagiarism of any kind is forbidden.

**Non-negotiable rules:**
- Direct quotes MUST use quotation marks and cite the source
- Paraphrased content MUST be substantially rewritten and cited
- Code adapted from external sources MUST include attribution comments in the code itself
- APA citation format MUST be used consistently throughout the book

**Rationale:** Academic integrity and legal compliance require proper attribution. Plagiarism undermines credibility and exposes the project to legal risk.

### VI. Deliverables and Tooling Constraints

The project MUST use Spec-Kit Plus methodology with Claude Code, Docusaurus for the book, and a RAG chatbot embedded in the final output.

**Non-negotiable rules:**
- Book content MUST be authored using Docusaurus with Markdown
- Source repository MUST follow Spec-Kit Plus structure (.specify/, specs/, history/)
- RAG chatbot MUST index the book content and answer queries using retrieval-augmented generation
- Final deliverables MUST include a PDF export and a GitHub Pages deployment
- Word count MUST be between 20,000 and 30,000 words
- At least 50 distinct sources MUST be cited

**Rationale:** These constraints ensure project manageability, consistency with tooling, and alignment with the defined scope. Spec-Kit Plus provides a structured workflow; Docusaurus enables maintainable documentation; RAG chatbot adds interactive learning value.

### VII. AI-to-Robot Pipeline Reproducibility

AI models, training scripts, deployment scripts, and robot control pipelines MUST be reproducible in the specified simulation and hardware environments.

**Non-negotiable rules:**
- Simulations MUST use ROS 2, Gazebo, Unity, or NVIDIA Isaac Sim as specified
- AI models MUST document training data sources, hyperparameters, and hardware requirements
- Deployment scripts MUST specify dependencies, environment setup, and validation steps
- Capstone task (autonomous humanoid execution) MUST have a reproducible validation protocol

**Rationale:** The book culminates in an autonomous humanoid capstone. Reproducibility ensures that the capstone is not just theoretical but practically achievable by readers following the guide.

## Citation Standards

**Minimum citation requirements:**
- At least 50 unique sources
- At least 50% peer-reviewed or official documentation
- APA format consistently applied
- Every code block adapted from external sources includes inline attribution

**Citation verification:**
- During planning and implementation, sources MUST be collected and validated
- Tools (linters, plagiarism checkers) MAY be used to verify citation compliance
- Human review MUST verify source quality and traceability

## Technical Reproducibility

**Environment specification:**
- Ubuntu 22.04 or later for ROS 2 and Gazebo
- Python 3.10+ for AI/ML code
- Specific versions for all libraries (e.g., TensorFlow 2.13, PyTorch 2.0)
- GPU requirements (e.g., NVIDIA RTX 3060 or equivalent for training)

**Validation protocol:**
- Each code sample MUST pass basic syntax and runtime checks
- Simulations MUST load without errors in the specified environment
- Capstone validation: autonomous humanoid completes defined tasks in simulation

## Deliverables & Scope

**Word count:** 20,000–30,000 words

**Format:**
- Docusaurus-based book (Markdown sources)
- PDF export for offline distribution
- GitHub Pages deployment for web access

**RAG Chatbot:**
- Embedded in the Docusaurus site
- Retrieves context from book chapters
- Answers user queries with citations to relevant sections

**Simulations:**
- ROS 2 + Gazebo for robotics fundamentals
- Unity or Isaac Sim for advanced scenarios
- All simulation worlds, robot models, and configuration files version-controlled

**Capstone:**
- Autonomous humanoid robot executing multi-step tasks
- Reproducible in simulation
- Optional: hardware deployment instructions for physical robots

## Governance

**Amendment procedure:**
- Constitution amendments require documented rationale and impact analysis
- Version increments follow semantic versioning:
  - MAJOR: Backward-incompatible changes (e.g., removing a principle)
  - MINOR: New principles or expanded guidance
  - PATCH: Clarifications, typo fixes, wording improvements
- All amendments MUST be approved before implementation work begins

**Compliance verification:**
- All pull requests MUST verify alignment with constitution principles
- Spec, plan, and task documents MUST include a "Constitution Check" section
- Violations MUST be justified in a "Complexity Tracking" section with rationale

**Runtime guidance:**
- See CLAUDE.md for agent-specific development guidelines
- See .specify/templates/ for document structure and workflow

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
