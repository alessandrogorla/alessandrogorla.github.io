import { lazy, Suspense, useCallback, useEffect } from "react";
import { FileText } from "lucide-react";
import useBreakpoint from "@hooks/useBreakpoint";
import useFocusTrap from "@hooks/useFocusTrap";
import ModalShell from "@components/ui/ModalShell";
import ModalHeaderShell from "@components/ui/ModalHeaderShell";
import { CYAN, TEXT_PRIMARY } from "@/constants/theme";

const CvDocument = lazy(() => import("./CvDocument"));

interface CvViewerModalProps {
   isOpen: boolean;
   onClose: () => void;
   fileUrl: string | null;
}

const CvViewerModal = ({ isOpen, onClose, fileUrl }: CvViewerModalProps) => {
   if (!isOpen || !fileUrl) return null;
   const { isMobile } = useBreakpoint();
   const dialogRef = useFocusTrap<HTMLDivElement>(isOpen);

   // Riconosce dinamicamente se è il Transcript o il CV in base all'URL
   const isTranscript = fileUrl.includes("carrer_certificate_polimi"); 
   const modalTitle = isTranscript ? "Transcript of Records" : "Curriculum Vitae";

   const onEsc = useCallback(
      (e: KeyboardEvent) => {
         if (e.key === "Escape") onClose();
      },
      [onClose],
   );

   useEffect(() => {
      if (!isOpen) return;
      document.body.style.overflow = "hidden";
      document.addEventListener("keydown", onEsc);
      return () => {
         document.body.style.overflow = "";
         document.removeEventListener("keydown", onEsc);
      };
   }, [isOpen, onEsc]);

   return (
      <ModalShell
         isOpen={isOpen}
         onClose={onClose}
         dialogRef={dialogRef}
         isMobile={isMobile}
         titleId="cv-viewer-title"
      >
         <ModalHeaderShell
            isMobile={isMobile}
            onClose={onClose}
            closeLabel="Close document viewer"
         >
            <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
               <FileText
                  size={isMobile ? 16 : 18}
                  style={{ color: CYAN, flexShrink: 0 }}
               />
               <h2
                  id="cv-viewer-title"
                  style={{
                     fontSize: isMobile ? 15 : 18,
                     fontWeight: 700,
                     color: TEXT_PRIMARY,
                  }}
               >
                  {modalTitle}
               </h2>
            </div>
         </ModalHeaderShell>

         {isOpen && (
            <Suspense
               fallback={
                  <div
                     className="skeleton"
                     style={{
                        margin: 16,
                        height: 420,
                        borderRadius: 8,
                     }}
                  />
               }
            >
               {/* QUI PASSIAMO CORRETTAMENTE IL FILE URL AL SOTTO-COMPONENTE */}
               <CvDocument isMobile={isMobile} fileUrl={fileUrl} />
            </Suspense>
         )}
      </ModalShell>
   );
};

export default CvViewerModal;