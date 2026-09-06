import { useMemo, useState } from "react";
import { ExternalLink, Download, ZoomIn, ZoomOut } from "lucide-react";
import { MONO_FONT, TEXT_MUTED, TEXT_SECONDARY } from "@/constants/theme";

const ZOOM_STEPS = [0.75, 1, 1.25, 1.5];

interface CvDocumentProps {
   isMobile: boolean;
   fileUrl: string | null; // <-- Riceve l'URL dinamico
}

const CvDocument = ({ isMobile, fileUrl }: CvDocumentProps) => {
   const [zoomIdx, setZoomIdx] = useState(1);

   const zoom = ZOOM_STEPS[zoomIdx];
   const zoomLabel = useMemo(() => `${Math.round(zoom * 100)}%`, [zoom]);

   // Usa il fileUrl ricevuto, oppure una stringa vuota di fallback se è null
   const currentUrl = fileUrl || "";

   // Determina il titolo e l'etichetta in base al documento aperto
   const isTranscript = currentUrl.includes("carrer_certificate_polimi");
   const docTitle = isTranscript ? "Transcript of Records" : "Curriculum Vitae";

   return (
      <div style={{ display: "flex", flexDirection: "column", minHeight: 0 }}>
         {/* Toolbar */}
         <div
            style={{
               display: "flex",
               alignItems: "center",
               justifyContent: "space-between",
               gap: 8,
               padding: isMobile ? "10px 14px" : "10px 20px",
               borderBottom: "1px solid rgba(255,255,255,0.06)",
            }}
         >
            <span
               style={{
                  fontFamily: MONO_FONT,
                  fontSize: 11,
                  color: TEXT_MUTED,
               }}
            >
               Local PDF
            </span>
            <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
               <button
                  onClick={() => setZoomIdx((i) => Math.max(0, i - 1))}
                  disabled={zoomIdx === 0}
                  aria-label="Zoom out"
                  className="btn-outline"
                  style={{
                     padding: "6px 10px",
                     opacity: zoomIdx === 0 ? 0.4 : 1,
                  }}
               >
                  <ZoomOut size={14} />
               </button>
               <span
                  style={{
                     fontFamily: MONO_FONT,
                     fontSize: 11,
                     color: TEXT_SECONDARY,
                     minWidth: 38,
                     textAlign: "center",
                  }}
               >
                  {zoomLabel}
               </span>
               <button
                  onClick={() =>
                     setZoomIdx((i) => Math.min(ZOOM_STEPS.length - 1, i + 1))
                  }
                  disabled={zoomIdx === ZOOM_STEPS.length - 1}
                  aria-label="Zoom in"
                  className="btn-outline"
                  style={{
                     padding: "6px 10px",
                     opacity: zoomIdx === ZOOM_STEPS.length - 1 ? 0.4 : 1,
                  }}
               >
                  <ZoomIn size={14} />
               </button>
               <a
                  href={currentUrl}
                  target="_blank"
                  rel="noopener noreferrer"
                  aria-label="Open document in a new tab"
                  className="btn-outline"
                  style={{
                     display: "inline-flex",
                     padding: "6px 10px",
                     textDecoration: "none",
                  }}
               >
                  <ExternalLink size={14} />
               </a>
               <a
                  href={currentUrl}
                  download
                  aria-label="Download document"
                  className="btn-primary"
                  style={{
                     display: "inline-flex",
                     alignItems: "center",
                     gap: 6,
                     padding: "6px 12px",
                     fontSize: 12,
                     textDecoration: "none",
                  }}
               >
                  <Download size={14} />
                  {!isMobile && "Download"}
               </a>
            </div>
         </div>

         {/* Pages / Viewer */}
         <div
            style={{
               overflow: "auto",
               padding: isMobile ? 10 : 16,
               display: "flex",
               flexDirection: "column",
               alignItems: "center",
               gap: 12,
               background: "#0a0f11",
            }}
         >
            <div
               style={{
                  width: "100%",
                  maxWidth: 860,
                  transform: `scale(${zoom})`,
                  transformOrigin: "top center",
               }}
            >
               <iframe
                  title={docTitle}
                  src={currentUrl}
                  style={{
                     width: "100%",
                     height: isMobile ? "72vh" : "78vh",
                     border: "none",
                     borderRadius: 8,
                     background: "#fff",
                     boxShadow: "0 8px 32px rgba(0,0,0,0.5)",
                  }}
               />
            </div>
         </div>
      </div>
   );
};

export default CvDocument;