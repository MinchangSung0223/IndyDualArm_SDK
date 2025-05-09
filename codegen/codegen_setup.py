import os
import re

def modify_cmakelists_and_headers(root_dir):
    for dirpath, dirnames, _ in os.walk(root_dir):
        for dirname in dirnames:
            if not dirname.endswith("_grt_rtw"):
                continue

            project_name = dirname.replace("_grt_rtw", "")
            cmake_path = os.path.join(dirpath, dirname, "CMakeLists.txt")
            header_path = os.path.join(dirpath, dirname, f"{project_name}.h")

            # ───────────────────────────────────────────────
            # Step 1. Modify CMakeLists.txt
            # ───────────────────────────────────────────────
            if os.path.isfile(cmake_path):
                with open(cmake_path, 'r') as f:
                    content = f.read()

                with open(cmake_path + ".bak", 'w') as f_backup:
                    f_backup.write(content)

                content = re.sub(r'\badd_excutable\b', 'add_library', content)  # fix typo
                pattern = rf'add_executable\(\s*{re.escape(project_name)}\b'
                replacement = f'add_library({project_name} STATIC'
                content, count = re.subn(pattern, replacement, content)

                with open(cmake_path, 'w') as f:
                    f.write(content)

                print(f"✅ {cmake_path} 수정 완료 ({count}건 변경됨)")

            # ───────────────────────────────────────────────
            # Step 2. Wrap extern C in header file
            # ───────────────────────────────────────────────
            if not os.path.isfile(header_path):
                continue

            with open(header_path, 'r') as f:
                lines = f.readlines()

            with open(header_path + ".bak", 'w') as f:
                f.writelines(lines)

            new_lines = []
            inside_model_entry_block = False
            buffer = []

            for line in lines:
                if '/* Model entry point functions */' in line:
                    inside_model_entry_block = True
                    new_lines.append(line)
                    continue

                # collect only the 3 target lines
                if inside_model_entry_block and re.match(r'\s*extern void \w+_(initialize|step|terminate)\(void\);', line):
                    buffer.append(re.sub(r'^\s*extern\s+', '', line))  # remove only 'extern'
                    continue

                # finish wrapping
                if inside_model_entry_block and buffer:
                    new_lines.append('#ifdef __cplusplus\nextern "C" {\n#endif\n')
                    new_lines.extend(buffer)
                    new_lines.append('#ifdef __cplusplus\n}\n#endif\n')
                    buffer.clear()
                    inside_model_entry_block = False

                new_lines.append(line)

            if buffer:
                print(f"⚠️ {header_path} - model entry point 3줄 감지 못함, 건너뜀")
                continue

            with open(header_path, 'w') as f:
                f.writelines(new_lines)

            print(f"✅ {header_path} wrapping 완료")

# ──────────────────────────────
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    modify_cmakelists_and_headers(current_dir)
