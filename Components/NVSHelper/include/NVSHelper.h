#ifndef NVSHELPER_
#define NVSHELPER_

void nvs_init();
void nvs_get(char *key, uint8_t *reciever, int size);
void nvs_set(char *key, uint8_t buffer);

#endif