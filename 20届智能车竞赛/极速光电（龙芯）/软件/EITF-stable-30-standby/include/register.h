#ifndef REGISTER_H
#define REGISTER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

// 页面大小
#define PAGE_SIZE 0x10000

// 寄存器读写宏定义
#define REG_READ(addr) (*(volatile uint32_t *)(addr))
#define REG_WRITE(addr, val) (*(volatile uint32_t *)(addr) = (val))

// 映射物理地址到虚拟地址的函数声明
void *map_register(uint32_t physical_address, size_t size);

#endif