import { describe, it, expect, vi } from "vitest";
import { render } from "@testing-library/react";

vi.mock("lenis", () => ({
   default: class {
      raf() {
         // no-op: test stub
      }
      destroy() {
         // no-op: test stub
      }
      scrollTo() {
         // no-op: test stub
      }
   },
}));

describe("App", () => {
   it("renders without crashing", async () => {
      const { default: App } = await import("../App");
      const { container } = render(<App />);
      expect(container.querySelector("main")).toBeTruthy();
   });
});

describe("Data files", () => {
   it("loads all projects", async () => {
      const { getProjects } = await import("@data/dataLoader");
      expect(getProjects().length).toBeGreaterThan(0);
   });

   it("every project has required fields", async () => {
      const { getProjects } = await import("@data/dataLoader");
      for (const p of getProjects()) {
         expect(p.id).toBeDefined();
         expect(p.title).toBeTruthy();
         expect(p.description).toBeTruthy();
         expect(p.github).toBeDefined();
         expect(Array.isArray(p.tools_tech)).toBe(true);
      }
   });

   it("loads certifications with valid badge fields", async () => {
      const { getCertifications } = await import("@data/dataLoader");
      const certs = getCertifications();
      expect(certs.length).toBeGreaterThan(0);
      for (const c of certs) {
         expect(c.badgeId).toBeTruthy();
         expect(c.badgeUrl).toContain("credly.com");
      }
   });
});
