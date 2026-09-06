import { useState, useMemo, useCallback } from "react";
import { getProjects } from "@data/dataLoader";
import useBreakpoint from "@hooks/useBreakpoint";
import { MAX_WIDTH } from "@/constants/theme";
import PageSection from "@components/layout/PageSection";
import { parseDate } from "./portfolioConstants";
import type { ProjectWithCategory } from "./portfolioConstants";
import ProjectGrid from "./ProjectGrid";
import OpenSourceBanner from "./OpenSourceBanner";
import ProjectModal from "./ProjectModal";

const Portfolio = () => {
   const [selectedProject, setSelectedProject] =
      useState<ProjectWithCategory | null>(null);
   const { isMobile } = useBreakpoint();

   const projects = useMemo(
      () =>
         [...getProjects()].sort(
         (a, b) => parseDate(b.date).getTime() - parseDate(a.date).getTime(),
         ),
      [],
   );

   const handleOpenProject = useCallback(
      (project: ProjectWithCategory) => setSelectedProject(project),
      [],
   );

   return (
      <PageSection id="projects" title="Projects" subtitle="Things I've built">
         <div style={{ maxWidth: MAX_WIDTH, margin: "0 auto" }}>
            {/* Card grid with live screenshots / animated covers */}
            <ProjectGrid
               projects={projects}
               isMobile={isMobile}
               onOpenProject={handleOpenProject}
            />

            {/* Open Source Contributions Banner */}
            {/* <OpenSourceBanner />*/}
         </div>

         <ProjectModal
            project={selectedProject}
            onClose={() => setSelectedProject(null)}
         />
      </PageSection>
   );
};

export default Portfolio;
