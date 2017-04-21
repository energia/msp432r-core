/*
 *  Copyright 2017 by Texas Instruments Incorporated.
 *
 */

/*
 *  ======== dwarf.h ========
 */
#ifndef DWARF_
#define DWARF_    1

typedef struct Dwarf_CUHeader32 {
    Bits32  unit_length;
    Bits16 version;
    Bits32 debug_abbrev_offset;
    Bits8 address_size;
} Dwarf_CUHeader32;

typedef struct Dwarf_CUHeader64 {
    Bits32  unit_length;
    Bits64  real_unit_length;
    Bits16 version;
    Bits64 debug_abbrev_offset;
    Bits8 address_size;
} Dwarf_CUHeader64;

typedef struct Dwarf_LNHeader32 {
    Bits32  unit_length;                /* total size of CU line # info */
    Bits16  version;                    /* version of line number info */
    Bits32  header_length;              /* offset to start of line # program */
    Bits8   minimum_instruction_length;
    //Bits8   maximum_operations_per_instruction;    // Dwarf 4 only
    Bits8   default_is_stmt;
    Bits8   line_base;
    Bits8   line_range;
    Bits8   opcode_base;
    Bits8   standard_opcode_lengths[1];  /* length = opcode_base - 1 */
    /* include_directories; '\0' terminated sequence of strings */
    /* file_names; string, LEB128[3] = file dir index, mod time, and len */
} Dwarf_LNHeader32;

#define Dwarf_LNHDR32SIZE 16

typedef struct Dwarf_LNHeader64 {
    Bits32  unit_length;
    Bits64  real_unit_length;
    Bits16  version;                    /* version of line number info */
    Bits64  header_length;              /* offset to start of line # program */
    Bits8   minimum_instruction_length;
    //Bits8   maximum_operations_per_instruction;    // Dwarf 4 only
    Bits8   default_is_stmt;
    Bits8   line_base;
    Bits8   line_range;
    Bits8   opcode_base;
    Bits8   standard_opcode_lengths[1];  /* length = opcode_base */
    /* include_directories; '\0' terminated sequence of strings */
    /* file_names; string, LEB128[3] = file dir index, mod time, and len */
} Dwarf_LNHeader64;

#define Dwarf_LNHDR64SIZE 24

typedef struct Dwarf_DieAttr {
    Bits32  id;
    Bits32  form;
} Dwarf_DieAttr;

typedef struct Dwarf_Abbrev {
    Bits32          code;
    Bits32          tag;
    Bool            children;
    Int             numAttrs;
    Dwarf_DieAttr   *attrs;     /* variable number of Dwarf_DieAttrs */
} Dwarf_Abbrev;

/*
 *  ======== attributes ========
 */
#define DW_AT_sibling       0x01 /* reference */
#define DW_AT_location      0x02 /* block, loclistptr */
#define DW_AT_name          0x03 /* string */

#define DW_AT_ordering      0x09 /* constant */

#define DW_AT_byte_size     0x0b /* block, constant, reference */
#define DW_AT_bit_offset    0x0c /* block, constant, reference */
#define DW_AT_bit_size      0x0d /* block, constant, reference */

#define DW_AT_stmt_list     0x10 /* lineptr */
#define DW_AT_low_pc        0x11 /* address */
#define DW_AT_high_pc       0x12 /* address */
#define DW_AT_language      0x13 /* constant */

#define DW_AT_discr         0x15 /* reference */
#define DW_AT_discr_value   0x16 /* constant */
#define DW_AT_visibility    0x17 /* constant */
#define DW_AT_import        0x18 /* reference */
#define DW_AT_string_length 0x19 /* block, loclistptr */
#define DW_AT_common_reference 0x1a /* reference */
#define DW_AT_comp_dir      0x1b /* string */
#define DW_AT_const_value   0x1c /* block, constant, string */
#define DW_AT_containing_type 0x1d /* reference */
#define DW_AT_default_value 0x1e /* reference */

#define DW_AT_inline        0x20 /* constant */
#define DW_AT_is_optional   0x21 /* flag */
#define DW_AT_lower_bound   0x22 /* block, constant, reference */

#define DW_AT_producer      0x25 /* string */

#define DW_AT_prototyped    0x27 /* flag */

#define DW_AT_return_addr   0x2a /* block, loclistptr */

#define DW_AT_start_scope   0x2c /* constant */

#define DW_AT_bit_stride    0x2e /* constant */
#define DW_AT_upper_bound   0x2f /* block, constant, reference */

#define DW_AT_abstract_origin 0x31 /* reference */
#define DW_AT_accessibility 0x32 /* constant */
#define DW_AT_address_class 0x33 /* constant */
#define DW_AT_artificial    0x34 /* flag */
#define DW_AT_base_types    0x35 /* reference */
#define DW_AT_calling_convention 0x36 /* constant */
#define DW_AT_count         0x37 /* block, constant, reference */
#define DW_AT_data_member_location 0x38 /* block, constant, loclistptr */
#define DW_AT_decl_column   0x39 /* constant */
#define DW_AT_decl_file     0x3a /* constant */
#define DW_AT_decl_line     0x3b /* constant */
#define DW_AT_declaration   0x3c /* flag */
#define DW_AT_discr_list    0x3d /* block */
#define DW_AT_encoding      0x3e /* constant */
#define DW_AT_external      0x3f /* flag */
#define DW_AT_frame_base    0x40 /* block, loclistptr */
#define DW_AT_friend        0x41 /* reference */
#define DW_AT_identifier_case 0x42 /* constant */
#define DW_AT_macro_info    0x43 /* macptr */
#define DW_AT_namelist_item 0x44 /* block */
#define DW_AT_priority      0x45 /* reference */
#define DW_AT_segment       0x46 /* block, loclistptr */
#define DW_AT_specification 0x47 /* reference */
#define DW_AT_static_link   0x48 /* block, loclistptr */
#define DW_AT_type          0x49 /* reference */
#define DW_AT_use_location  0x4a /* block, loclistptr */
#define DW_AT_variable_parameter 0x4b /* flag */
#define DW_AT_virtuality    0x4c /* constant */
#define DW_AT_vtable_elem_location 0x4d /* block, loclistptr */
#define DW_AT_allocated     0x4e /* block, constant, reference */
#define DW_AT_associated    0x4f /* block, constant, reference */
#define DW_AT_data_location 0x50 /* block */
#define DW_AT_byte_stride   0x51 /* block, constant, reference */
#define DW_AT_entry_pc      0x52 /* address */
#define DW_AT_use_UTF8      0x53 /* flag */
#define DW_AT_extension     0x54 /* reference */
#define DW_AT_ranges        0x55 /* rangelistptr */
#define DW_AT_trampoline    0x56 /* address, flag, reference, string */
#define DW_AT_call_column   0x57 /* constant */
#define DW_AT_call_file     0x58 /* constant */
#define DW_AT_call_line     0x59 /* constant */
#define DW_AT_description   0x5a /* string */
#define DW_AT_binary_scale  0x5b /* constant */
#define DW_AT_decimal_scale 0x5c /* constant */
#define DW_AT_small         0x5d /* reference */
#define DW_AT_decimal_sign  0x5e /* constant */
#define DW_AT_digit_count   0x5f /* constant */
#define DW_AT_picture_string 0x60 /* string */
#define DW_AT_mutable       0x61 /* flag */
#define DW_AT_threads_scaled 0x62 /* flag */
#define DW_AT_explicit      0x63 /* flag */
#define DW_AT_object_pointer 0x64 /* reference */
#define DW_AT_endianity     0x65 /* constant */
#define DW_AT_elemental     0x66 /* flag */

#define DW_AT_lo_user       0x2000
#define DW_AT_hi_user       0x3fff

/*
 *  ======== tags ========
 */
#define DW_TAG_array_type               0x01
#define DW_TAG_class_type               0x02
#define DW_TAG_entry_point              0x03
#define DW_TAG_enumeration_type         0x04
#define DW_TAG_formal_parameter         0x05
#define DW_TAG_imported_declaration     0x08
#define DW_TAG_label                    0x0a
#define DW_TAG_lexical_block            0x0b
#define DW_TAG_member                   0x0d
#define DW_TAG_pointer_type             0x0f
#define DW_TAG_reference_type           0x10
#define DW_TAG_compile_unit             0x11
#define DW_TAG_string_type              0x12
#define DW_TAG_structure_type           0x13
#define DW_TAG_subroutine_type          0x15
#define DW_TAG_typedef                  0x16
#define DW_TAG_union_type               0x17
#define DW_TAG_unspecified_parameters   0x18
#define DW_TAG_variant                  0x19
#define DW_TAG_common_block             0x1a
#define DW_TAG_common_inclusion         0x1b
#define DW_TAG_inheritance              0x1c
#define DW_TAG_inlined_subroutine       0x1d
#define DW_TAG_module                   0x1e
#define DW_TAG_ptr_to_member_type       0x1f
#define DW_TAG_set_type                 0x20
#define DW_TAG_subrange_type            0x21
#define DW_TAG_with_stmt                0x22
#define DW_TAG_access_declaration       0x23
#define DW_TAG_base_type                0x24
#define DW_TAG_catch_block              0x25
#define DW_TAG_const_type               0x26
#define DW_TAG_constant                 0x27
#define DW_TAG_enumerator               0x28
#define DW_TAG_file_type                0x29
#define DW_TAG_friend                   0x2a
#define DW_TAG_namelist                 0x2b
#define DW_TAG_namelist_item            0x2c
#define DW_TAG_packed_type              0x2d
#define DW_TAG_subprogram               0x2e
#define DW_TAG_template_type_parameter  0x2f
#define DW_TAG_template_value_parameter 0x30
#define DW_TAG_thrown_type              0x31
#define DW_TAG_try_block                0x32
#define DW_TAG_variant_part             0x33
#define DW_TAG_variable                 0x34
#define DW_TAG_volatile_type            0x35
#define DW_TAG_dwarf_procedure          0x36
#define DW_TAG_restrict_type            0x37
#define DW_TAG_interface_type           0x38
#define DW_TAG_namespace                0x39
#define DW_TAG_imported_module          0x3a
#define DW_TAG_unspecified_type         0x3b
#define DW_TAG_partial_unit             0x3c
#define DW_TAG_imported_unit            0x3d
#define DW_TAG_condition                0x3f
#define DW_TAG_shared_type              0x40

#define DW_TAG_lo_user                  0x4080
#define DW_TAG_hi_user                  0xffff

/*
 *  ======== forms ========
 */
#define DW_FORM_addr      0x01 /* address */
#define DW_FORM_block2    0x03 /* block */
#define DW_FORM_block4    0x04 /* block */
#define DW_FORM_data2     0x05 /* constant */
#define DW_FORM_data4     0x06 /* constant, lineptr, loclistptr, macptr, rangelistptr */
#define DW_FORM_data8     0x07 /* constant, lineptr, loclistptr, macptr, rangelistptr */
#define DW_FORM_string    0x08 /* string */
#define DW_FORM_block     0x09 /* block */
#define DW_FORM_block1    0x0a /* block */
#define DW_FORM_data1     0x0b /* constant */
#define DW_FORM_flag      0x0c /* flag */
#define DW_FORM_sdata     0x0d /* constant */
#define DW_FORM_strp      0x0e /* string */
#define DW_FORM_udata     0x0f /* constant */
#define DW_FORM_ref_addr  0x10 /* reference */
#define DW_FORM_ref1      0x11 /* reference */
#define DW_FORM_ref2      0x12 /* reference */
#define DW_FORM_ref4      0x13 /* reference */
#define DW_FORM_ref8      0x14 /* reference */
#define DW_FORM_ref_udata 0x15 /* reference */
#define DW_FORM_indirect  0x16 /* see Section 7.5.  */

/*
 *  ======== Dwarf_getAttrName ========
 */
extern String Dwarf_getAttrName(Bits32 attr);

/*
 *  ======== Dwarf_getFormName ========
 */
extern String Dwarf_getFormName(Bits32 form);

/*
 *  ======== Dwarf_getFormSize ========
 */
Int Dwarf_getFormSize(Bits32 form);

/*
 *  ======== Dwarf_getTagName ========
 */
extern String Dwarf_getTagName(Bits32 tag);

/*
 *  ======== Dwarf_setFormAddrSize ========
 */
extern Void Dwarf_setFormAddrSize(Int size);

#endif
/*
 *  @(#) ti.targets.omf.elf; 1,0,0,0; 2-10-2017 09:26:13; /db/ztree/library/trees/xdctargets/xdctargets-m11/src/ xlibrary

 */

