#ifndef __KG_LOG_H__
#define __KG_LOG_H__

#if IS_ENABLED(CONFIG_KG_LOG)
extern int __init kg_log_init(void);
extern void kg_log(const char *fmt, ...);
extern void kg_log_u64_array(char *str, unsigned long data[], int size);
extern void kg_log_print(struct seq_file *m);

#define KG_LOG(fmt, ...) do { \
		pr_info(fmt, ##__VA_ARGS__); \
		kg_log(fmt, ##__VA_ARGS__); \
	} while (0)
#else
static inline int kg_log_init(void)
{
	return 0;
}

static inline void kg_log_u64_array(char *str, unsigned long data[], int size)
{
	int i;

	pr_cont("%s", str);

	for (i = 0; i < size; i++) {
		pr_cont("%lu", data[i]);

		if (i < (size - 1))
			pr_cont(",");
	}

	pr_cont("\n");
}

static inline void kg_log_print(struct seq_file *m)
{
	return;
}

#define KG_LOG(fmt, ...) pr_info(fmt, ##__VA_ARGS__)
#endif

#endif /* __KG_LOG_H__ */
