import { Calendar, Users } from "lucide-react";
import ModalHeaderShell from "@components/ui/ModalHeaderShell";
import { TEXT_PRIMARY, TEXT_MUTED, MONO_FONT } from "@/constants/theme";
import type { CategoryColors, ProjectWithCategory } from "./portfolioConstants";

interface ProjectModalHeaderProps {
   project: ProjectWithCategory;
   colors: CategoryColors;
   isMobile: boolean;
   onClose: () => void;
}

const ProjectModalHeader = ({
   project,
   colors,
   isMobile,
   onClose,
}: ProjectModalHeaderProps) => {
   return (
      <ModalHeaderShell
         isMobile={isMobile}
         onClose={onClose}
         closeLabel="Close project details"
      >
         <div
            style={{
               display: "flex",
               alignItems: "center",
               marginBottom: 4,
            }}
         >
            <h2
               id="project-modal-title"
               style={{
                  fontSize: isMobile ? 15 : 18,
                  fontWeight: 700,
                  color: TEXT_PRIMARY,
                  flex: 1,
                  minWidth: 0,
                  overflow: "hidden",
                  textOverflow: "ellipsis",
                  whiteSpace: "nowrap",
               }}
            >
               {project.title}
            </h2>
         </div>

         <div
            style={{
               display: "flex",
               alignItems: "center",
               gap: 12,
               flexWrap: "wrap",
            }}
         >
            <span
               style={{
                  display: "inline-flex",
                  alignItems: "center",
                  gap: 4,
                  fontSize: isMobile ? 11 : 12,
                  color: colors.accent,
                  fontFamily: MONO_FONT,
               }}
            >
               <Calendar size={11} style={{ flexShrink: 0 }} />
               {project.date} - {project.organization}
            </span>
            {project.team && (
               <span
                  style={{
                     display: "inline-flex",
                     alignItems: "center",
                     gap: 4,
                     fontSize: isMobile ? 11 : 12,
                     color: TEXT_MUTED,
                     fontFamily: MONO_FONT,
                  }}
               >
                  <Users size={11} style={{ flexShrink: 0 }} />
                  {project.team}
               </span>
            )}
         </div>
      </ModalHeaderShell>
   );
};

export default ProjectModalHeader;
