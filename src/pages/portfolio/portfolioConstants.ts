import {
   getOpenSourceContributions,
   getCommunityDiscussions,
} from "@data/dataLoader";
import type {
   Project,
   OpenSourceContribution,
   CommunityDiscussion,
} from "@/types";

export const MONTHS: Record<string, number> = {
   January: 0,
   February: 1,
   March: 2,
   April: 3,
   May: 4,
   June: 5,
   July: 6,
   August: 7,
   September: 8,
   October: 9,
   November: 10,
   December: 11,
};

export const parseDate = (dateStr: string): Date => {
   const [month, year] = dateStr.split(" ");
   const y = Number(year);
   // ?? (not ||) so a valid "January" (index 0) isn't treated as missing.
   return new Date(Number.isFinite(y) ? y : 0, MONTHS[month] ?? 0);
};

export interface CategoryColors {
   accent: string;
   gradient: string;
   bgAlpha: string;
   borderAlpha: string;
}

export const PROJECT_COLORS: CategoryColors = {
   accent: "#60a5fa",
   gradient: "linear-gradient(to right, #60a5fa, #3b82f6)",
   bgAlpha: "rgba(96,165,250,",
   borderAlpha: "rgba(96,165,250,",
};

export const getProjectColors = (): CategoryColors => PROJECT_COLORS;

/** True for real project URLs -- excludes empty strings and the "#" placeholder used in JSON data. */
export const isValidUrl = (url: string | undefined): url is string =>
   !!url && url !== "#";

export type ProjectWithCategory = Project;

export const OPEN_SOURCE_CONTRIBUTIONS: OpenSourceContribution[] =
   getOpenSourceContributions();

export const COMMUNITY_DISCUSSIONS: CommunityDiscussion[] =
   getCommunityDiscussions();
