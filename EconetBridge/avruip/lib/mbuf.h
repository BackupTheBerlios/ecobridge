#ifndef MBUF_H
#define MBUF_H

#include <stdint.h>
#include <stdlib.h>

#define MBUF_SIZE	128

struct mbuf
{
  struct mbuf *next, *prev;
  uint16_t length;
  uint8_t data[MBUF_SIZE];
};

extern struct mbuf *mbuf_alloc(void);
extern void mbuf_free(struct mbuf *);

extern struct mbuf *copy_to_mbufs(uint8_t *data, uint16_t length);

#endif
