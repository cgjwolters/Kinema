#pragma once

extern "C" __declspec(dllexport) void *NewModel(const wchar_t *name);

extern "C" __declspec(dllexport) void *NewArcLinTrack(bool trkClosed = false, double trackPipeDiameter = 0.0);
