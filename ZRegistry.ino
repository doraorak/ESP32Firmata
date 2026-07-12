// Module registry + dispatch. This file's name makes it sort LAST in Arduino's
// alphabetical .ino concatenation, so every module instance defined in the
// sibling files (irModule, sonarModule, dhtModule, displayModule) is already
// visible here. The abstract ModuleHandler base lives in the main sketch (which
// always concatenates first). Adding a module = one .ino + one array entry.

static ModuleHandler *const modules[] = { &irModule, &sonarModule, &dhtModule, &displayModule };
static const uint8_t MODULE_COUNT = sizeof(modules) / sizeof(modules[0]);

static void moduleDispatch(uint8_t id, const uint8_t *payload, int length) {
  for (uint8_t i = 0; i < MODULE_COUNT; i++)
    if (modules[i]->id() == id) { modules[i]->handle(payload, length); return; }
}

static void moduleTick() {
  for (uint8_t i = 0; i < MODULE_COUNT; i++) modules[i]->tick();
}

static void moduleReset() {
  for (uint8_t i = 0; i < MODULE_COUNT; i++) modules[i]->reset();
}

static void handleModuleData(const uint8_t *data, int length) {
  if (length < 1) return;
  if (data[0] == MODULE_QUERY) {
    int index = 0;
    frameBuf[index++] = START_SYSEX;
    frameBuf[index++] = MODULE_DATA;
    frameBuf[index++] = MODULE_LIST_REPLY;
    frameBuf[index++] = MODULE_COUNT;
    for (uint8_t moduleIndex = 0; moduleIndex < MODULE_COUNT; moduleIndex++) {
      frameBuf[index++] = modules[moduleIndex]->id();
      frameBuf[index++] = modules[moduleIndex]->major();
      frameBuf[index++] = modules[moduleIndex]->minor();
      const char *moduleName = modules[moduleIndex]->name();
      uint8_t nameLength = (uint8_t)strlen(moduleName);
      frameBuf[index++] = nameLength;
      for (uint8_t charIndex = 0; charIndex < nameLength; charIndex++)
        frameBuf[index++] = moduleName[charIndex] & 0x7F;
    }
    frameBuf[index++] = END_SYSEX;
    sendFrame(frameBuf, index);
    return;
  }
  moduleDispatch(data[0], data + 1, length - 1);
}
