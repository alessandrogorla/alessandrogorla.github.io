import { Calendar, Users } from "lucide-react";
import { MONO_FONT, TEXT_PRIMARY, TEXT_MUTED } from "@/constants/theme";
import type { CategoryColors, ProjectWithCategory } from "./portfolioConstants";

interface ProjectCardHeaderProps {
   data: ProjectWithCategory;
   colors: CategoryColors;
}

const ProjectCardHeader = ({
   data,
   colors,
}: ProjectCardHeaderProps) => (
   <>
      {/* Header: title and metadata */}
      <div
         style={{
            display: "flex",
            alignItems: "flex-start",
            marginBottom: 12,
         }}
      >
         <div style={{ minWidth: 0, flex: 1 }}>
            <h3
               style={{
                  fontSize: 16,
                  fontWeight: 700,
                  color: TEXT_PRIMARY,
                  lineHeight: 1.2,
               }}
            >
               {data.title}
            </h3>
            <div
               style={{
                  display: "flex",
                  alignItems: "center",
                  gap: 8,
                  marginTop: 4,
                  flexWrap: "wrap",
               }}
            >
               <span
                  style={{
                     display: "inline-flex",
                     alignItems: "center",
                     gap: 4,
                     fontSize: 11,
                     color: TEXT_MUTED,
                     fontFamily: MONO_FONT,
                  }}
               >
                  <Calendar size={10} style={{ flexShrink: 0 }} />
                  {data.date}
               </span>
               {data.team && (
                  <span
                     style={{
                        display: "inline-flex",
                        alignItems: "center",
                        gap: 4,
                        fontSize: 11,
                        color: colors.accent,
                        fontFamily: MONO_FONT,
                     }}
                  >
                     <Users size={10} style={{ flexShrink: 0 }} />
                     {data.team}
                  </span>
               )}
            </div>
         </div>
      </div>

   </>
);

export default ProjectCardHeader;
