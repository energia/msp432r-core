/*
 *  Copyright 2017 by Texas Instruments Incorporated.
 *
 */

/*
 *  ======== dwarf.c ========
 */
#include <xdc/std.h>
#include <stdio.h>

#include "dwarf.h"

static String tagNames[] = {
    "null",                         /* 0x00 */
    "DW_TAG_array_type",            /* 0x01 */
    "DW_TAG_class_type",            /* 0x02 */
    "DW_TAG_entry_point",           /* 0x03 */
    "DW_TAG_enumeration_type",      /* 0x04 */
    "DW_TAG_formal_parameter",      /* 0x05 */
    "0x06",                         /* 0x06 */
    "0x07",                         /* 0x07 */
    "DW_TAG_imported_declaration",  /* 0x08 */
    "0x09",                         /* 0x09 */
    "DW_TAG_label",                 /* 0x0a */
    "DW_TAG_lexical_block",         /* 0x0b */
    "0x0c",                         /* 0x0c */
    "DW_TAG_member",                /* 0x0d */
    "0x0e",                         /* 0x0e */
    "DW_TAG_pointer_type",          /* 0x0f */
    "DW_TAG_reference_type",        /* 0x10 */
    "DW_TAG_compile_unit",          /* 0x11 */
    "DW_TAG_string_type",           /* 0x12 */
    "DW_TAG_structure_type",        /* 0x13 */
    "0x14",                         /* 0x14 */
    "DW_TAG_subroutine_type",       /* 0x15 */
    "DW_TAG_typedef",               /* 0x16 */
    "DW_TAG_union_type",            /* 0x17 */
    "DW_TAG_unspecified_parameters",/* 0x18 */
    "DW_TAG_variant",               /* 0x19 */
    "DW_TAG_common_block",          /* 0x1a */
    "DW_TAG_common_inclusion",      /* 0x1b */
    "DW_TAG_inheritance",           /* 0x1c */
    "DW_TAG_inlined_subroutine",    /* 0x1d */
    "DW_TAG_module",                /* 0x1e */
    "DW_TAG_ptr_to_member_type",    /* 0x1f */
    "DW_TAG_set_type",              /* 0x20 */
    "DW_TAG_subrange_type",         /* 0x21 */
    "DW_TAG_with_stmt",             /* 0x22 */
    "DW_TAG_access_declaration",    /* 0x23 */
    "DW_TAG_base_type",             /* 0x24 */
    "DW_TAG_catch_block",           /* 0x25 */
    "DW_TAG_const_type",            /* 0x26 */
    "DW_TAG_constant",              /* 0x27 */
    "DW_TAG_enumerator",            /* 0x28 */
    "DW_TAG_file_type",             /* 0x29 */
    "DW_TAG_friend",                /* 0x2a */
    "DW_TAG_namelist",              /* 0x2b */
    "DW_TAG_namelist_item",         /* 0x2c */
    "DW_TAG_packed_type",           /* 0x2d */
    "DW_TAG_subprogram",            /* 0x2e */
    "DW_TAG_template_type_parameter",  /* 0x2f */
    "DW_TAG_template_value_parameter", /* 0x30 */
    "DW_TAG_thrown_type",           /* 0x31 */
    "DW_TAG_try_block",             /* 0x32 */
    "DW_TAG_variant_part",          /* 0x33 */
    "DW_TAG_variable",              /* 0x34 */
    "DW_TAG_volatile_type",         /* 0x35 */
    "DW_TAG_dwarf_procedure",       /* 0x36 */
    "DW_TAG_restrict_type",         /* 0x37 */
    "DW_TAG_interface_type",        /* 0x38 */
    "DW_TAG_namespace",             /* 0x39 */
    "DW_TAG_imported_module",       /* 0x3a */
    "DW_TAG_unspecified_type",      /* 0x3b */
    "DW_TAG_partial_unit",          /* 0x3c */
    "DW_TAG_imported_unit",         /* 0x3d */
    "0x3e",                         /* 0x3e */
    "DW_TAG_condition",             /* 0x3f */
    "DW_TAG_shared_type",           /* 0x40 */
    "DW_TAG_type_unit",             /* 0x41 (Dwarf4) */
    "DW_TAG_rvalue_reference_type", /* 0x42 (Dwarf4) */
    "DW_TAG_template_alias",        /* 0x43 (Dwarf4) */
};

static String attrNames[] = {
    "null",                     /* 0x00 */
    "DW_AT_sibling",            /* 0x01 (CU) reference */
    "DW_AT_location",           /* 0x02 */
    "DW_AT_name",               /* 0x03 (CU) string */
    "???",
    "???",
    "???",
    "???",
    "???",
    "DW_AT_ordering",           /* 0x09 */
    "???",
    "DW_AT_byte_size",          /* 0x0b */
    "DW_AT_bit_offset",         /* 0x0c */
    "DW_AT_bit_size",           /* 0x0d */
    "???",
    "???",
    "DW_AT_stmt_list",          /* 0x10 (CU) lineptr */
    "DW_AT_low_pc",             /* 0x11 (CU) address */
    "DW_AT_high_pc",            /* 0x12 (CU) address */
    "DW_AT_language",           /* 0x13 (CU) constant (LEB128) */
    "???",
    "DW_AT_discr",              /* 0x15 */
    "DW_AT_discr_value",        /* 0x16 */
    "DW_AT_visibility",         /* 0x17 */
    "DW_AT_import",             /* 0x18 */
    "DW_AT_string_length",      /* 0x19 */
    "DW_AT_common_reference",    /* 0x1a */
    "DW_AT_comp_dir",           /* 0x1b (CU) string */
    "DW_AT_const_value",        /* 0x1c */
    "DW_AT_containing_type",    /* 0x1d */
    "DW_AT_default_value",      /* 0x1e */
    "???",
    "DW_AT_inline",             /* 0x20 */
    "DW_AT_is_optional",        /* 0x21 */
    "DW_AT_lower_bound",        /* 0x22 */
    "???",
    "???",
    "DW_AT_producer",           /* 0x25 (CU) string */
    "???",
    "DW_AT_prototyped",         /* 0x27 */
    "???",
    "???",
    "DW_AT_return_addr",        /* 0x2a */
    "???",
    "DW_AT_start_scope",        /* 0x2c */
    "???",
    "DW_AT_bit_stride",         /* 0x2e */
    "DW_AT_upper_bound",        /* 0x2f */
    "???",
    "DW_AT_abstract_origin",    /* 0x31 */
    "DW_AT_accessibility",      /* 0x32 */
    "DW_AT_address_class",      /* 0x33 */
    "DW_AT_artificial",         /* 0x34 */
    "DW_AT_base_types",         /* 0x35 (CU) reference */
    "DW_AT_calling_convention", /* 0x36 */
    "DW_AT_count",              /* 0x37 */
    "DW_AT_data_member_location",/* 0x38 */
    "DW_AT_decl_column",        /* 0x39 */
    "DW_AT_decl_file",          /* 0x3a */
    "DW_AT_decl_line",          /* 0x3b */
    "DW_AT_declaration",        /* 0x3c */
    "DW_AT_discr_list",         /* 0x3d */
    "DW_AT_encoding",           /* 0x3e */
    "DW_AT_external",           /* 0x3f */
    "DW_AT_frame_base",         /* 0x40 */
    "DW_AT_friend",             /* 0x41 */
    "DW_AT_identifier_case",    /* 0x42 (CU) constant */
    "DW_AT_macro_info",         /* 0x43  macptr (CU) */
    "DW_AT_namelist_item",      /* 0x44  block */
    "DW_AT_priority",           /* 0x45  reference */
    "DW_AT_segment",            /* 0x46  block, loclistptr */
    "DW_AT_specification",      /* 0x47  reference */
    "DW_AT_static_link",        /* 0x48  block, loclistptr */
    "DW_AT_type",               /* 0x49  reference */
    "DW_AT_use_location",       /* 0x4a  block, loclistptr */
    "DW_AT_variable_parameter", /* 0x4b  flag */
    "DW_AT_virtuality",         /* 0x4c  constant */
    "DW_AT_vtable_elem_location",/* 0x4d  block, loclistptr */
    "DW_AT_allocated",          /* 0x4e  block, constant, reference */
    "DW_AT_associated",         /* 0x4f  block, constant, reference */
    "DW_AT_data_location",      /* 0x50  block */
    "DW_AT_byte_stride",        /* 0x51  block, constant, reference */
    "DW_AT_entry_pc",           /* 0x52  address */
    "DW_AT_use_UTF8",           /* 0x53  flag */
    "DW_AT_extension",          /* 0x54  reference */
    "DW_AT_ranges",             /* 0x55  rangelistptr */
    "DW_AT_trampoline",         /* 0x56  address, flag, reference, string */
    "DW_AT_call_column",        /* 0x57  constant */
    "DW_AT_call_file",          /* 0x58  constant */
    "DW_AT_call_line",          /* 0x59  constant */
    "DW_AT_description",        /* 0x5a  string */
    "DW_AT_binary_scale",       /* 0x5b  constant */
    "DW_AT_decimal_scale",      /* 0x5c  constant */
    "DW_AT_small",              /* 0x5d  reference */
    "DW_AT_decimal_sign",       /* 0x5e  constant */
    "DW_AT_digit_count",        /* 0x5f  constant */
    "DW_AT_picture_string",     /* 0x60  string */
    "DW_AT_mutable",            /* 0x61  flag */
    "DW_AT_threads_scaled",     /* 0x62  flag */
    "DW_AT_explicit",           /* 0x63  flag */
    "DW_AT_object_pointer",     /* 0x64  reference */
    "DW_AT_endianity",          /* 0x65  constant */
    "DW_AT_elemental",          /* 0x66  flag */
    "DW_AT_pure",               /* 0x67  flag */
    "DW_AT_recursive",          /* 0x68  flag */
    "DW_AT_signature",          /* 0x69  reference (Dwarf4) */
    "DW_AT_main_subprogram",    /* 0x6a  flag (Dwarf4) */
    "DW_AT_data_bit_offset",    /* 0x6b  constant (Dwarf4) */
    "DW_AT_const_expr",         /* 0x6c  flag (Dwarf4) */
    "DW_AT_enum_class",         /* 0x6d  flag (Dwarf4) */
    "DW_AT_linkage_name",       /* 0x6e string (Dwarf4) */
};

typedef struct Form {
    String name;        /* name of form (DW_FORM_*) */
    Int    size;        /* size of form in bytes */
} Form;

static Form forms[] = {
    {"null", -1},
    {"DW_FORM_addr", -1},    /* 0x01,  address (size defined in CU header) */
    {"0x02", -1},            /* 0x02 */
    {"DW_FORM_block2", 2},   /* 0x03,  block */
    {"DW_FORM_block4", 4},   /* 0x04,  block */
    {"DW_FORM_data2", 2},    /* 0x05,  constant */
    {"DW_FORM_data4", 4},    /* 0x06,  constant, lineptr, loclistptr, macptr, rangelistptr */
    {"DW_FORM_data8", 8},    /* 0x07,  constant, lineptr, loclistptr, macptr, rangelistptr */
    {"DW_FORM_string", 0},   /* 0x08,  string ('\0' terminated chars) */
    {"DW_FORM_block", 0},    /* 0x09,  block (unsigned LEB128) */
    {"DW_FORM_block1", 1},   /* 0x0a,  block */
    {"DW_FORM_data1", 1},    /* 0x0b,  constant */
    {"DW_FORM_flag", 1},     /* 0x0c,  flag */
    {"DW_FORM_sdata", 0},    /* 0x0d,  constant (signed LEB128) */
    {"DW_FORM_strp", 4},     /* 0x0e,  string (8 for 64-bit dwarf) */
    {"DW_FORM_udata", 0},    /* 0x0f,  constant (unsigned LEB128) */
    {"DW_FORM_ref_addr", 4}, /* 0x10,  reference (8 for 64-bit dwarf) */
    {"DW_FORM_ref1", 1},     /* 0x11,  reference */
    {"DW_FORM_ref2", 2},     /* 0x12,  reference */
    {"DW_FORM_ref4", 4},     /* 0x13,  reference */
    {"DW_FORM_ref8", 8},     /* 0x14,  reference */
    {"DW_FORM_ref_udata", 0},/* 0x15,  reference (unsigned LEB128) */
    {"DW_FORM_indirect", 0}, /* 0x16,  see Section 7.5. (unsigned LEB128) */
};

/*
 *  ======== Dwarf_getAttrName ========
 */
String Dwarf_getAttrName(Bits32 attr)
{
    static Char dynamicName[48];

    if (attr < (sizeof (attrNames) / sizeof (String))) {
        return (attrNames[attr]);
    }
    else if (attr >= DW_AT_lo_user && attr <= DW_AT_hi_user) {
        sprintf(dynamicName, "DW_AT_user_%.4X", attr);
    }
    else {
        sprintf(dynamicName, "<unknown DW_AT value: %.4X>", attr);
    }
    return (dynamicName);
}

/*
 *  ======== Dwarf_getFormName ========
 */
String Dwarf_getFormName(Bits32 form)
{
    static Char dynamicName[48];

    if (form < (sizeof (forms) / sizeof (Form))) {
        return (forms[form].name);
    }
    else {
        sprintf(dynamicName, "<unknown DW_FORM value: %.4X>", form);
    }
    return (dynamicName);
}

/*
 *  ======== Dwarf_getFormSize ========
 */
Int Dwarf_getFormSize(Bits32 form)
{
    if (form < (sizeof (forms) / sizeof (Form))) {
        return (forms[form].size);
    }
    return (-1);
}

/*
 *  ======== Dwarf_getTagName ========
 */
String Dwarf_getTagName(Bits32 tag)
{
    static Char dynamicName[48];

    if (tag < (sizeof (tagNames) / sizeof (String))) {
        return (tagNames[tag]);
    }
    else if (tag >= DW_TAG_lo_user && tag <= DW_TAG_hi_user) {
        sprintf(dynamicName, "DW_TAG_user_%.4X", tag);
    }
    else {
        sprintf(dynamicName, "<unknown DW_TAG value: %.4X>", tag);
    }
    return (dynamicName);
}

/*
 *  ======== Dwarf_setFormAddrSize ========
 */
Void Dwarf_setFormAddrSize(Int size)
{
    forms[DW_FORM_addr].size = size;
}
/*
 *  @(#) ti.targets.omf.elf; 1,0,0,0; 2-10-2017 09:26:13; /db/ztree/library/trees/xdctargets/xdctargets-m11/src/ xlibrary

 */

