#ifndef ATTRIBUTE_TYPEDEF_H
#define ATTRIBUTE_TYPEDEF_H

#ifndef __packed__  // 取消结构体成员之间的对齐，使结构体紧凑排列。
#define __packed__ __attribute__((packed))
#endif

#ifndef __aligned__  // 指定变量或结构体的对齐方式，以字节为单位。
#define __aligned__(x) __attribute__((aligned(x)))
#endif

#ifndef __section__  // 指定变量或函数存放在特定的段中。
#define __section__(x) __attribute__((section(x)))
#endif

#ifndef __weak__  // 标记为弱符号，如果有多个同名符号，优先使用强符号。
#define __weak__ __attribute__((weak))
#endif

#ifndef __noreturn__  // 标记函数不返回。
#define __noreturn__ __attribute__((noreturn))
#endif

#ifndef __naked__  // 指定函数不使用函数调用的方式，直接跳转到函数的入口地址。
#define __naked__ __attribute__((naked))
#endif

#ifndef __unused__  // 标记为未使用的函数或变量，防止编译器产生警告。
#define __unused__ __attribute__((unused))
#endif

#ifndef __used__  // 标记为使用的函数或变量。
#define __used__ __attribute__((used))
#endif

#ifndef __deprecated__  // 标记为过时的函数或变量，在使用时会产生警告。。
#define __deprecated__ __attribute__((deprecated))
#endif

#ifndef __always_inline__  // 强制内联函数，即使-O0编译选项也会内联。
#define __always_inline__ __attribute__((always_inline))
#endif

#ifndef __format__  // 格式化检查，用于检查printf、scanf等函数的参数格式是否正确。
#define __format__ __attribute__((format))
#endif

#endif  // ATTRIBUTE_TYPEDEF_H